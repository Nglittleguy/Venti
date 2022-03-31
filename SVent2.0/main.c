/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_uart.h"
#include "app_util.h"
#include "ble_gap.h"
#include "ble_nus.h"
#include "ble_db_discovery.h"
#include "ble_nus_c.h"
#include "nrf_ble_scan.h"

//Connection handle of vent to relay to
static uint16_t m_conn_relay = BLE_CONN_HANDLE_INVALID;

//Connection handle array
static uint16_t m_conn_handles[NRF_SDH_BLE_PERIPHERAL_LINK_COUNT] = {BLE_CONN_HANDLE_INVALID};

//******************************************
#include "venti.h"

APP_TIMER_DEF(m_time_segment_id);
APP_TIMER_DEF(m_start_id);


//Tracking information for current time, segment, and day in file
static uint32_t current_epoch_sec = 0;
static uint16_t current_time_segment = 0;     //Sun 12am
uint16_t dataFileID = 0;                      //Can't be 0xFFFF

bool to_write_voltage = false;

//Buffers to read messages from NUS (copy because destructive parsing)
uint8_t data_read[200] = {0};
uint8_t data_read_buf[200] = {0};

//Buffer to send file records to user
uint8_t data_flash_send_to_phone[768] = {0};

//Buffer to send return NUS messages to user
uint8_t return_buf[200] = {0};
uint16_t return_len = 0;


//Buffer to write to flash (asynchronous) must not be on stack
static char flash_write_buf[8];
static char flash_write_temp[8];
static char flash_write_schedule[128];


/**@brief Function to wait for fds availability
 *
 * @details Do nothing until fds is initialized
 */
static void wait_for_fds_ready() {
    while(!m_fds_initialized) {
        sd_app_evt_wait();
    }
}

static void record_day_voltage();
static void record_delete(uint32_t fid, uint32_t key);

/**@brief Function to handle events from the Flash Data Storage
 *
 * @details Validates proper fds usage after an event occurs
 *
 * @param[in]   p_evt         flash data storage event
 */
static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->result == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Event: %s received (NRF_SUCCESS)",
                      fds_evt_str[p_evt->id]);
    }
    else
    {
        NRF_LOG_INFO("Event: %s received (%s)",
                      fds_evt_str[p_evt->id],
                      fds_err_str(p_evt->result));
    }

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }

            //After temperature write, then write record of day-voltage record
            if(to_write_voltage) {
                to_write_voltage = false;
                record_day_voltage();
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
            
        } break;

        default:
            break;
    }
}


/**@brief Function to Initialize Flash Data Storage via Default Config File
 *
 * @details Initializes FDS
 */
static void flash_storage_init() {
    fds_stat_t stat = {0};
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    (void) fds_register(fds_evt_handler);
    ret_code_t err = fds_init();
    APP_ERROR_CHECK(err);

    wait_for_fds_ready();
    err= fds_stat(&stat);
    APP_ERROR_CHECK(err);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    err = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);
    if(err==NRF_SUCCESS) {
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        err = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(err);

        /* Copy the configuration from flash into m_dummy_cfg. */
        memcpy(&m_dummy_cfg, config.p_data, sizeof(configuration_t));

        NRF_LOG_INFO("Config file found, updating boot count to %d.", m_dummy_cfg.boot_count);

        /* Update boot count. */
        m_dummy_cfg.boot_count++;

        /* Close the record when done reading. */
        err = fds_record_close(&desc);
        APP_ERROR_CHECK(err);

        /* Write the updated record to flash. */
        err = fds_record_update(&desc, &m_dummy_record);
        if ((err != NRF_SUCCESS) && (err == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err);
        }
    }
    else {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing config file...");

        err = fds_record_write(&desc, &m_dummy_record);
        if ((err != NRF_SUCCESS) && (err == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(err);
        }
    }
}

/**@brief Function to Read from Flash Records and Return to User (Deprecated)
 *
 * @details Reads record from flash of Record File ID, and Record Key
 *
 * @param[in]   fid           file ID to write
 * @param[in]   key           record key to write
 * @param[in]   conn_to_send  connection handle to send info to
 * @param[in]   from_central  return should be sent to central connection
 */
static void record_read_file(uint16_t fileID, uint16_t recordKey, uint16_t conn_to_send, bool from_central) {

    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};
    fds_flash_record_t rec;

    //Set buffer index to keep track of multiple records
    uint16_t bufferIndex = 0, i = 0;
    memset(data_flash_send_to_phone, 0, 768);

    while(fds_record_find(fileID, recordKey, &desc, &tok)==NRF_SUCCESS) {
        if(fds_record_open(&desc, &rec)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in opening record");
        }
        
        //Read data as char array (string)
        uint8_t* raw_data = (uint8_t *)rec.p_data;
        NRF_LOG_INFO("Record: %s\n\r", raw_data);

        i = 0;
        //Loop through until end of string, set into buffer to send to phone
        while(raw_data[i]!='\0') {
            data_flash_send_to_phone[bufferIndex] = raw_data[i];
            i++;
            bufferIndex++;
        }
        data_flash_send_to_phone[bufferIndex] = '\n';
        bufferIndex++;


        if(fds_record_close(&desc)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in closing record");
        }
    }
    NRF_LOG_INFO("Sent: %s\n\r", data_flash_send_to_phone);

    ret_code_t err_code;

    //Send to Central
    if(from_central) {
        ble_nus_c_string_send(&m_ble_nus_c, data_flash_send_to_phone, bufferIndex);
        if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    //Send to Peripheral
    else {
        for(int i = 0; i<NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
            if(m_conn_handles[i] == conn_to_send) {
                err_code = ble_nus_data_send(&m_nus, data_flash_send_to_phone, &bufferIndex, m_conn_handles[i]);
                if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
        }
    }
    
}

/**@brief Function to Write to Flash Records
 *
 * @details Writes record into flash of Record File ID, and Record Key
 *
 * @param[in]   fid           file ID to write
 * @param[in]   key           record key to write
 * @param[in]   p_data        data to be written to flash
 * @param[in]   len           number of bytes to write to flash (word aligned)
 */
static void record_write(uint32_t fid, uint32_t key, void const * p_data, uint32_t len)
{
    fds_record_t const rec =
    {
        .file_id           = fid,
        .key               = key,
        .data.p_data       = p_data,
        .data.length_words = (len + 3) / sizeof(uint32_t)
    };

    NRF_LOG_INFO("writing record to flash...\nfile: 0x%x, key: 0x%x, \"%s\", len: %u bytes\n", fid, key, p_data, len);

    ret_code_t err = fds_record_write(NULL, &rec);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("error: fds_record_write() returned %s.\n", fds_err_str(err));
    }
}

/**@brief Function to Delete Flash Records
 *
 * @details Deletes all records with matching Record File ID, and Record Key
 *
 * @param[in]   fid           file ID to delete
 * @param[in]   key           record key to delete
 */
static void record_delete(uint32_t fid, uint32_t key)
{
    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};

    if (fds_record_find(fid, key, &desc, &tok) == NRF_SUCCESS)
    {
        ret_code_t rc = fds_record_delete(&desc);
        if (rc != NRF_SUCCESS)
        {
            NRF_LOG_INFO("error: fds_record_delete() returned %s.\n", fds_err_str(rc));
            return;
        }
        NRF_LOG_INFO("record id: 0x%x\n", desc.record_id);
    }
    else
    {
        NRF_LOG_INFO("error: record not found!\n");
    }
}


/**@brief Function to clear out flash memory
 *
 * @details Clears out deleted flash memory to be reused
 */
static void fds_garbage_collection()
{
    ret_code_t rc = fds_gc();
    switch (rc)
    {
        case NRF_SUCCESS:
            break;

        default:
            NRF_LOG_INFO("error: garbage collection returned %s\n", fds_err_str(rc));
            break;
    }
}

/**@brief Function to read and load the schedule from flash
 *
 * @details Loads schedule from flash - if success, return 1, else 0
 */
static bool record_load_schedule() {
    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};
    fds_flash_record_t rec;

    //Look for records with key and ID designated for schedule records
    while(fds_record_find(SCHEDULE_FLASH_ID, SCHEDULE_FLASH_KEY, &desc, &tok)==NRF_SUCCESS) {
        if(fds_record_open(&desc, &rec)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in opening record");
        }
        
        void* raw_data = (void *)rec.p_data;

        //Copy the memory directly into the Schedule_event array
        memcpy(schedule, raw_data, sizeof(Schedule_event)*35);

        return true;
        
        if(fds_record_close(&desc)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in closing record");
        }
    }
    return false;
}

/**@brief Function to record the schedule when requested
 *
 * @details Calls function to set the schedule into the buffer, write into flash
 *
 */
static void record_schedule() {
    setScheduleBuffer(flash_write_schedule);
    record_write(SCHEDULE_FLASH_ID, SCHEDULE_FLASH_KEY, &flash_write_schedule, 128);
}

/**@brief Function to Return Day-Voltage Records
 *
 * @details Reads voltage record of the day, sends it back to user
 *
 * @param[in]   fileID        File ID of the day of voltage records
 * @param[in]   conn_to       Connection Handle of sender (to respond back to)
 * @param[in]   from_central  Identifying which type of sender - central or peripheral connection
 */
static void record_read_day(uint16_t fileID, uint16_t conn_to_send, bool from_central) {

    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};
    fds_flash_record_t rec;

    //Index buffer for noting number of chars to write as string - clear buffer, start with header
    uint16_t bufferIndex = 7, i = 0;
    memset(data_flash_send_to_phone, 0, 768);
    sprintf(data_flash_send_to_phone, "00@945@");

    //Look for records with key 1 (designated for voltage records)
    while(fds_record_find(fileID, 0x0001, &desc, &tok)==NRF_SUCCESS) {
        if(fds_record_open(&desc, &rec)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in opening record");
        }
        
        //Read data in as uint32_t array
        uint32_t* raw_data = (uint32_t *)rec.p_data;
        
        //Check the length required as string to send to user
        i = snprintf(NULL, 0, "%d|%d", raw_data[0], raw_data[1]);
        
        //Write that many bytes - note the number in the index
        sprintf(data_flash_send_to_phone+bufferIndex, "%d|%d", raw_data[0], raw_data[1]);
        bufferIndex += i;

        //Add a \n for easier reading and parsing
        data_flash_send_to_phone[bufferIndex] = '#';
        bufferIndex++;


        if(fds_record_close(&desc)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in closing record");
        }
    }
    NRF_LOG_INFO("Sent: %s\n\r", data_flash_send_to_phone);

    ret_code_t err_code;

    //Send to Central
    if(from_central) {
        ble_nus_c_string_send(&m_ble_nus_c, data_flash_send_to_phone, bufferIndex);
        if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    //Send to Peripheral
    else {
        for(int i = 0; i<NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
            if(m_conn_handles[i] == conn_to_send) {
                err_code = ble_nus_data_send(&m_nus, data_flash_send_to_phone, &bufferIndex, m_conn_handles[i]);
                if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
        }
    }
    
}

/**@brief Function to record the new day and voltage of the batteries
 *
 * @details Calls function to update the day count, write the current voltage and time into flash
 *
 */
static void record_day_voltage() {
    dataFileID++;
    setDayVoltageBuffer(flash_write_buf, current_epoch_sec);
    record_write(dataFileID, 0x0001, &flash_write_buf, 8);
}

/**@brief Function to Return Temperature Records
 *
 * @details Reads temperature records of the day, combines them, sends them back to user
 *
 * @param[in]   fileID        File ID of the day of temperature records
 * @param[in]   conn_to       Connection Handle of sender (to respond back to)
 * @param[in]   from_central  Identifying which type of sender - central or peripheral connection
 */
static void record_read_temp(uint16_t fileID, uint16_t conn_to_send, bool from_central) {
    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};
    fds_flash_record_t rec;

    //Index buffer for noting number of chars to write as string - clear buffer, start with header
    uint16_t bufferIndex = 7, i = 0;
    memset(data_flash_send_to_phone, 0, 768);
    sprintf(data_flash_send_to_phone, "00@925@");

    //Look for records with key 2 (designated for temperature records)
    while(fds_record_find(fileID, 0x0002, &desc, &tok)==NRF_SUCCESS) {

        if(fds_record_open(&desc, &rec)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in opening record");
        }
        
        //Read in as short array (optimal storage capabilities)
        short* raw_data = (short *)rec.p_data;
        
        //Check the length required as string to send to user
        i = snprintf(NULL, 0, "%d|%d|%d", raw_data[0], raw_data[1], raw_data[2]);

        //Write that many bytes - note the number in the index
        sprintf(data_flash_send_to_phone+bufferIndex, "%d|%d|%d", raw_data[0], raw_data[1], raw_data[2]);
        bufferIndex += i;

        //Add a \n for easier reading and parsing
        data_flash_send_to_phone[bufferIndex] = '\n';
        bufferIndex++;

        if(fds_record_close(&desc)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in closing record");
        }
    }

    //Add a # for end
    data_flash_send_to_phone[bufferIndex] = '#';
    bufferIndex++;

    NRF_LOG_INFO("Sent: %s\n\r", data_flash_send_to_phone);
    ret_code_t err_code;

    //Send to Central
    if(from_central) {
        ble_nus_c_string_send(&m_ble_nus_c, data_flash_send_to_phone, bufferIndex);
        if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    //Send to Peripheral
    else {
        for(int i = 0; i<NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
            if(m_conn_handles[i] == conn_to_send) {
                err_code = ble_nus_data_send(&m_nus, data_flash_send_to_phone, &bufferIndex, m_conn_handles[i]);
                if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
        }
    }
}

/**@brief Function to record the temperature into flash storage
 *
 * @details Calls function to set the static buffer with time segment and temperature min/max
 *
 */
static void record_temp() {
    setTemperatureBuffer(flash_write_temp, current_time_segment);
    record_write(dataFileID, 0x0002, &flash_write_temp, 8);
}


//******************************************

#define MANUFACTURER_NAME               "SmartLabs"                             /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                0                                       /**< The advertising duration to continue until stop in units of 10 milliseconds. 0 means never stop*/
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */


// Connection Intervals to Change if Power Consumption Necessitates
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE);   /**< BLE GATT Queue instance. */
   
static void advertising_start();
static void application_timers_start(void);



/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */

// YOUR_JOB: Use UUIDs for service(s) used in your application.

static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function to handle the start timer
 *
 * @details Starts applications timers to synchronize clock with world
 *
 * @param[in]   p_context       Unused Parameter of App Timers
 */
static void start_timeout_handler(void *p_context) {
    UNUSED_PARAMETER(p_context);
    application_timers_start();
}

/**@brief Function to handle the time segment timer
 *
 * @details Updates the current time, checks the schedule, records information
 *
 * @param[in]   p_context       Unused Parameter of App Timers
 */
static void time_segment_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    current_time_segment++;
    current_epoch_sec+=60;

    //New Hour 
    if(current_time_segment % 12 == 0) {

        //New Day
        if(current_time_segment % 288 == 0) {

            //New Week, reset to 0
            if(current_time_segment == 2016) {
                current_time_segment = 0;
            } 
            
            //Flag to write voltage and current day later (FDS_EVT_WRITE)
            to_write_voltage = true;
        }

        //Write to file temperature min, max - then reset
        record_temp();
        resetTemperatureMinMax();
    }
    else {
        //Set the current min and max temperature of the hour
        compareTemperatureMinMax();
    }

    //Check the schedule - returns 1 if an event occured, else 0
    if(checkSchedule(current_time_segment)) {
        NRF_LOG_INFO("Schedule rotated the vent");
    }
}

/**@brief Function to relay the message to the Central connection
 *
 * @details Takes a message and sends it to through NUS Central, if a central is connected
 *
 * @param[in]   msg           String to be sent to the central connection
 */
static void relay(char* msg) {
    if(m_ble_nus_c.conn_handle!= BLE_CONN_HANDLE_INVALID && m_ble_nus_c.conn_handle!= 0) {

        uint8_t msg_len = strlen(msg);
        ret_code_t ret_val = ble_nus_c_string_send(&m_ble_nus_c, msg, msg_len);
        
        if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
        {
            APP_ERROR_CHECK(ret_val);
        }
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    //Repeated timer to keep track of time
    err_code = app_timer_create(&m_time_segment_id, APP_TIMER_MODE_REPEATED, time_segment_timeout_handler);
    APP_ERROR_CHECK(err_code);

    //Single timer to initialize time on device - start up at 5min
    err_code = app_timer_create(&m_start_id, APP_TIMER_MODE_SINGLE_SHOT, start_timeout_handler);
    APP_ERROR_CHECK(err_code);


    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    for(int i = 0; i<NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
        if(m_conn_handles[i] == p_evt->conn_handle && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
            m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        }
    }

    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

///**@snippet [Handling the data received over UART] */

/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            NRF_LOG_ERROR("Communication error occurred while handling UART.");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
    
    //Turn off UART pins (don't actually need output)
    NRF_UART0->TASKS_STOPRX=1;
    NRF_UART0->TASKS_STOPTX=1;
}
/**@snippet [UART Initialization] */


/**@brief Function to handle NUS messages
 *
 * @details This function is called to parse BLE NUS messages and respond accordingly
 *
 * @param[in]   data_in       Data of the message
 * @param[in]   data_len      Length of message in bytes
 * @param[in]   conn_to       Connection Handle of sender (to respond back to)
 * @param[in]   from_central  Identifying which type of sender - central or peripheral connection
 */

static void msg_received(const uint8_t* data_in, uint16_t data_len, uint16_t conn_to, bool from_central) {
    uint32_t err_code;
    ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();

    memset(data_read, '\0', 200);
    for (uint32_t i = 0; i < data_len && i < 200; i++) {
        data_read[i] = data_in[i];
    }

    //Debug and Useful To Reset Motor if Misaligned and Restarted
    if(strncmp(data_read, "r", 1)==0) {
        uint8_t target = (uint8_t) atoi(data_read+2);
        rotateCW(target);
    }

    else if(strncmp(data_read, "l", 1)==0) {
        uint8_t target = (uint8_t) atoi(data_read+2);
        rotateCCW(target);
    }
    
    //Set up relay
    else if((strcmp(data_read, "@@@@")==0 && !from_central)) {
        m_conn_relay = conn_to;
        NRF_LOG_INFO("Conn Handle of Relaying Node is %d", m_conn_relay);
    }

    //Coded Input 'id@cmd@param'
    else if(data_read[2]=='@' && data_read[6]=='@') {
        memset(data_read_buf, 0, 200);
        memcpy(data_read_buf, data_read, 200);
        NRF_LOG_INFO("Received: %s", data_read_buf);
        char *ptr = strtok(data_read, "@");
        uint8_t recipientID = atoi(ptr);

        //check to see if this device is the intended recipient, if not, relay
        if(DEVICE_ID != recipientID) {

            //attempt to send to central connection
            if(!from_central) {
                relay(data_read_buf);
            }
        
            //if recipient is 0, then send to all connected except sender
            if(recipientID == 0) {
                for(int i = 0; i<conn_handles.len; i++) {
                    if(conn_handles.conn_handles[i] != conn_to) {
                        err_code = ble_nus_data_send(&m_nus, data_read_buf, &data_len, conn_handles.conn_handles[i]);
                        NRF_LOG_INFO("Sending to %d", i);
                        if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                        if (err_code==NRF_ERROR_RESOURCES) {
                            i--;
                        }
                    }
                }
            }

            //if not from RIGHT relay node, send to RIGHT relay node (continue relay forward)
            else if(m_conn_relay != BLE_CONN_HANDLE_INVALID && m_conn_relay != conn_to) {
            
                err_code = ble_nus_data_send(&m_nus, data_read_buf, &data_len, m_conn_relay);
                if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
        }

        //Intended for this device
        else {
            ptr = strtok(NULL, "@");
            uint16_t instr_code = atoi(ptr);

            memset(return_buf, 0, 200);
            return_len = 0;
            
            //INSTRUCTION HANDLING HERE
            //-------------------------------------------------------------

            //Request Initial Set Up of Time (500@DayOfWeek|Hour|Minute|Second|EpochTime#)
            if(instr_code == 500) {
                ptr = strtok(NULL, "|");
                uint8_t instr_day_of_week = atoi(ptr);
                ptr = strtok(NULL, "|");
                uint8_t instr_hour = atoi(ptr);
                ptr = strtok(NULL, "|");
                uint8_t instr_minute = atoi(ptr);
                ptr = strtok(NULL, "|");
                uint8_t instr_second = atoi(ptr);
                ptr = strtok(NULL, "#");
                uint32_t instr_epoch = atoi(ptr);

                if(instr_day_of_week<7 && instr_hour<24 && instr_minute<60 && instr_second<60) {

                    //Find number of seconds until next segment
                    uint8_t seconds_to_next_minute = 60-instr_second;
                    uint16_t seconds_to_next_segment = seconds_to_next_minute;

                    if((instr_minute+1)%5!=0) {
                        seconds_to_next_segment += (5-(instr_minute+1)%5)*60;
                    }
                    
                    current_epoch_sec = instr_epoch + seconds_to_next_segment;

                    //Find Current Time Segment
                    current_time_segment = 288*instr_day_of_week + 12*instr_hour + (instr_minute/5)+1;
                    char dateTime[50] = {0};
                    currentTimeFromSegment(dateTime, current_time_segment);
                    NRF_LOG_INFO("Current Time Segment Set To %s, Epoch time: %d, Next Start in %d Seconds", dateTime, current_epoch_sec, seconds_to_next_segment);

                    err_code = app_timer_start(m_start_id, APP_TIMER_TICKS(seconds_to_next_segment*1000), NULL);
                    APP_ERROR_CHECK(err_code);

                    record_day_voltage();
                    sprintf(return_buf, "00@505@%s#", dateTime);
                }
                else {
                    sprintf(return_buf, "00@505@INVALID#");
                }
            }

            //Request Current Time Segment & Epoch Time
            else if(instr_code == 507) {
                char dateTime[50] = {0};
                currentTimeFromSegment(dateTime, current_time_segment);
                sprintf(return_buf, "00@508@%d|%s|%d#", current_time_segment, dateTime, current_epoch_sec);
            }

            //Set Current Time Segment
            else if(instr_code == 503) {
                ptr = strtok(NULL, "#");
                if (ptr!=NULL) {
                    uint16_t newTimeSegment = atoi(ptr);
                    if(newTimeSegment<2016) {
                        current_time_segment = newTimeSegment;
                    }
                }
                sprintf(return_buf, "00@504@%d#", current_time_segment);
            }

            //Request Battery Voltage (returns id@515@voltage_in_mV#)
            else if(instr_code == 510) {
                uint16_t vbatt;
                voltage_read_enable(true);
                battery_voltage_get(&vbatt);
                voltage_read_enable(false);

                //Low Battery (returns id@520@voltage_in_mV#)
                if(vbatt<4000 && !low_battery) {
                    sprintf(return_buf, "00@520@%d#", vbatt);
                }
                //Very Low Battery - Vent is Fully Open "Out of Service"
                else if(low_battery) {
                    sprintf(return_buf, "00@525@#", vbatt);
                }
                //Normal Battery 
                else {
                    sprintf(return_buf, "00@515@%d#", vbatt);
                }
            }

            //Request Temperature (returns id@535@temperature_in_C#)
            else if(instr_code == 530) {
                float current_temp = ds18b20_read_temp();
                sprintf(return_buf, "00@535@%f#", current_temp);
            }

            //Request Open Amount (returns id@605@open_amount_max_255#)
            else if(instr_code == 600) {
                sprintf(return_buf, "00@605@%d#", getOpenAmount());
            }

            //Request Open Vent to Amount (returns id@625#) - would recommend sending back amount as well 
            else if(instr_code == 620) {
                if(low_battery) {
                    sprintf(return_buf, "00@625@LOW#");
                }
                else {
                    ptr = strtok(NULL, "#");
                    uint8_t target_open = atoi(ptr);
                    rotate(target_open);
                    sprintf(return_buf, "00@625@#");
                }
            }

            //Request Schedules per Weekday (returns id@instr_code@time_segment|amount,time_segment|amount...#)
            else if(instr_code >= 711 && instr_code <= 717) {
                uint8_t weekday = instr_code-711;
                sendSchedule(weekday, return_buf);
                printf(return_buf);
            }
            
            //Request to Set Schedule per Weekday (returns id@instr_code@#)
            else if(instr_code >= 721 && instr_code <= 727) {
                //Set arrays of event times and amounts to set into schedule
                uint8_t weekday = instr_code-721;
                uint16_t timeArr[5] = {9999};
                uint8_t amountArr[5] = {255};
                for(int i = 0; i<5; i++) {
                    ptr = strtok(NULL, "|");
                    if (ptr!=NULL) {
                        timeArr[i] = atoi(ptr);
                        printf("%s ", ptr);
                    }
                    ptr = strtok(NULL, ",#");
                    if (ptr!=NULL) {
                        amountArr[i] = atoi(ptr);
                    }
                }
                setSchedule(weekday, timeArr, amountArr);
                sprintf(return_buf, "00@%d@#", weekday+721);
            }

            //Request Saving the Entire Schedule into Flash
            else if(instr_code == 750) {
                record_schedule();
                sprintf(return_buf, "00@755@#");
            }

            //Request Loading the Prior Schedule From Flash
            else if(instr_code == 760) {
                record_load_schedule();
                sprintf(return_buf, "00@765@#");
            }

            //Request Last Day of File ID
            else if(instr_code == 910) {
                sprintf(return_buf, "00@915@%d#", dataFileID);
            }

            //Set the File ID day
            else if(instr_code == 930) {
                ptr = strtok(NULL, "#");
                if (ptr!=NULL) {
                    dataFileID = atoi(ptr)-1;
                    record_day_voltage();
                }
                sprintf(return_buf, "00@935@#");
            }

            //Request Read of Flash File Temperature
            else if(instr_code == 920) {
                ptr = strtok(NULL, "#");
                if (ptr!=NULL) {
                    uint16_t fileID = atoi(ptr);
                    record_read_temp(fileID, conn_to, from_central);
                }
            }

            //Request Read of Flash File Day & Voltage
            else if(instr_code == 940) {
                ptr = strtok(NULL, "#");
                if (ptr!=NULL) {
                    uint16_t fileID = atoi(ptr);
                    record_read_day(fileID, conn_to, from_central);
                }
            }

            //Request Invalidate Delete of Flash File (Ready for Garbage Collection)
            else if(instr_code == 950) {
                ptr = strtok(NULL, "#");
                if (ptr!=NULL) {
                    uint16_t fileID = atoi(ptr);
                    if(fileID == SCHEDULE_FLASH_ID) {
                        record_delete(SCHEDULE_FLASH_ID, SCHEDULE_FLASH_KEY);
                    }
                    else {
                        record_delete(fileID, 0x0002);
                        record_delete(fileID, 0x0001);
                    }
                }
                sprintf(return_buf, "00@955@#");
            }

            //Request Garbage Collection of Flash Files
            else if(instr_code == 990) {
                fds_garbage_collection();
                sprintf(return_buf, "00@995@#");
            }


            // END INSTRUCTION HANDLING ----------------------------------
            
            return_len = strlen(return_buf);

            if(return_len > 0) {
                //Send response back to central connection
                if(from_central) {
                    err_code = ble_nus_c_string_send(&m_ble_nus_c, return_buf, return_len);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                }
                //Send response back to peripheral connection
                else {
                    err_code = ble_nus_data_send(&m_nus, return_buf, &return_len, conn_to);
                    if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                }
            }
        }
    }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        //Send the message over to message handling for both peripheral and central
        msg_received(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length, p_evt->conn_handle, false);
    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    //Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT +  NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init);
        APP_ERROR_CHECK(err_code);
    }

    APP_ERROR_CHECK(err_code);


    battery_voltage_init();



    //Initialize UART/NUS
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    initSchedule();

    /* YOUR_JOB: Add code to initialize the services used by the application.
       ble_xxs_init_t                     xxs_init;
       ble_yys_init_t                     yys_init;

       // Initialize XXX Service.
       memset(&xxs_init, 0, sizeof(xxs_init));

       xxs_init.evt_handler                = NULL;
       xxs_init.is_xxx_notify_supported    = true;
       xxs_init.ble_xx_initial_value.level = 100;

       err_code = ble_bas_init(&m_xxs, &xxs_init);
       APP_ERROR_CHECK(err_code);

       // Initialize YYY Service.
       memset(&yys_init, 0, sizeof(yys_init));
       yys_init.evt_handler                  = on_yys_evt;
       yys_init.ble_yy_initial_value.counter = 0;

       err_code = ble_yy_service_init(&yys_init, &yy_init);
       APP_ERROR_CHECK(err_code);
     */
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       ret_code_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */

    uint32_t err_code;
    err_code = app_timer_start(m_time_segment_id, SEGMENT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            printf("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    uint16_t evt_conn_handle =  p_ble_evt->evt.gap_evt.conn_handle;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // Remove conn_handle from list of disconnect
            for(int i = 0; i<NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
                if(m_conn_handles[i] == evt_conn_handle) {
                    m_conn_handles[i] = BLE_CONN_HANDLE_INVALID;
                }
            }
            // Set the relay connection handle to invalid
            if(m_conn_relay == evt_conn_handle) {
                m_conn_relay = BLE_CONN_HANDLE_INVALID;
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;
        
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONNECTED:
            // Connected
            // If connection is via central
            if (p_gap_evt->params.connected.role == BLE_GAP_ROLE_CENTRAL) {
                err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
                APP_ERROR_CHECK(err_code);

                // start discovery of services. The NUS Client waits for a discovery result
                err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
                APP_ERROR_CHECK(err_code);
            }
            // If connection is peripheral
            else if(p_gap_evt->params.connected.role == BLE_GAP_ROLE_PERIPH) {
                err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
                APP_ERROR_CHECK(err_code);
                uint32_t periph_link_cnt = ble_conn_state_peripheral_conn_count();
                ble_gap_evt_adv_report_t p_connected = p_gap_evt->params.adv_report;
                
                NRF_LOG_INFO("Connecting to %x", p_connected.peer_addr.addr);
                
                // Assign connection handle to available instance of QWR module.
                for (uint32_t i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++)
                {
                    if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
                    {
                        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], evt_conn_handle);
                        APP_ERROR_CHECK(err_code);
                        break;
                    }
                }
                
                // Assign m_conn_handles to keep record of all conn_handles
                // (deprecated use, should use
                //    ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();
                // instead)
                for (int i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
                    if (m_conn_handles[i] == BLE_CONN_HANDLE_INVALID) {
                        m_conn_handles[i] = evt_conn_handle;
                    }
                }

                // If the peripheral count is less than allowed, continue advertising
                if(periph_link_cnt != NRF_SDH_BLE_PERIPHERAL_LINK_COUNT) {
                    advertising_start();
                }
                break;
            }
            
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }

}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            for (int i = 0; i < NRF_SDH_BLE_PERIPHERAL_LINK_COUNT; i++) {
                if (m_conn_handles[i] != BLE_CONN_HANDLE_INVALID) {
                    err_code = sd_ble_gap_disconnect(m_conn_handles[i], BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                }

                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            
            break; // BSP_EVENT_DISCONNECT
     
        /*
          Sample on how to implement button I/O 
        */
        //case BSP_EVENT_KEY_3:
        //    NRF_LOG_INFO("Button pressed\n");
        //    do {
        //      uint8_t data_array[10] = "Pressed\n";
        //      uint16_t length = (uint16_t)10;
        //      err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handles[0]);
        //      if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
        //      {
        //        APP_ERROR_CHECK(err_code);
        //      }
        //    } while (err_code == NRF_ERROR_RESOURCES);
        //break;
        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start()
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

////******************************************
//// Central NUS Services

static ble_uuid_t const m_nus_uuid =
{
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
};


/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function to start scanning. */
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    
    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             scan_start();
         } break;

         default:
             break;
    }
}


/**@brief Function for initializing the central scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    //Set the filter for central scanning (name allocated based on DEVICE_ID)
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, RIGHT_NAME);
    APP_ERROR_CHECK(err_code);

    //Enable the filter for central scanning
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
    
}


/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Connected to device with Nordic UART Service.");

            //Send a coded message to allocate this node as a vent
            char* connect_msg = "@@@@";
            uint16_t connect_msg_len = sizeof(connect_msg);
            ble_nus_c_string_send(&m_ble_nus_c, connect_msg, connect_msg_len);
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:

            NRF_LOG_INFO("Received data from peripheral.");
            uint16_t len = p_ble_nus_evt->data_len;

            //Send the message over to message handling for both peripheral and central
            msg_received(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len, p_ble_nus_evt->conn_handle, true);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */


/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing and starting all Central activities. */
void app_nus_client_init()
{
    db_discovery_init();
    nus_c_init();
    scan_init();
    scan_start();
}

////******************************************


/**@brief Function for application main entry.
 */
int main(void)
{
	
    log_init();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    uart_init();
    services_init();
    advertising_init();
    conn_params_init();
    board_motor_init();
    flash_storage_init();
    advertising_start();

    //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    if(DEVICE_ID != 1) {
        app_nus_client_init();
    }
    
    for (;;)
    {
        idle_state_handle();;
    }
}