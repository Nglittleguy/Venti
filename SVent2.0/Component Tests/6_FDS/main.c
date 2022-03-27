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

#include "venti.h"

//Buffer to write to flash (asynchronous) must not be on stack
static char flash_write_buf[100];


/**@brief Function to wait for fds availability
 *
 * @details Do nothing until fds is initialized
 */
static void wait_for_fds_ready() {
    while(!m_fds_initialized) {
        sd_app_evt_wait();
    }
}

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

/**@brief Function to Read from Flash Records and Print it out
 *
 * @details Reads record from flash of Record File ID, and Record Key
 *
 * @param[in]   fid           file ID to write
 * @param[in]   key           record key to write
 */
static void record_read_file(uint16_t fileID, uint16_t recordKey) {
    NRF_LOG_INFO("Trying to read");
    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};
    fds_flash_record_t rec;
    
    while(fds_record_find(fileID, recordKey, &desc, &tok)==NRF_SUCCESS) {
        if(fds_record_open(&desc, &rec)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in opening record");
        }

        NRF_LOG_INFO("Record: %s\n\r", rec.p_data);

        if(fds_record_close(&desc)!=NRF_SUCCESS) {
            NRF_LOG_INFO("Error in closing record");
        }
    }
}

/**@brief Function to Write to Flash Records
 *
 * @details Writes record into flash of Record File ID, and Record Key
 *
 * @param[in]   fid           file ID to write
 * @param[in]   key           record key to write
 * @param[in]   len           length of bytes to write
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


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for application main entry.
 */
int main(void)
{
    log_init();
    flash_storage_init();
    
    memset(flash_write_buf, 0, 100);
    sprintf(flash_write_buf, "This Test Was Written by Stephen Ng on Mar 27, 2022 for Smartlabs Ltd.");

    uint16_t fileID = 1;
    uint16_t fileKey = 1;
    
    //Records are empty (Nothing at File 1, Key 1)
    NRF_LOG_INFO("Record should be empty");
    record_read_file(fileID, fileKey);

    nrf_delay_ms(1000);

    //Write into 
    NRF_LOG_INFO("Writing Record");
    record_write(fileID, fileKey, flash_write_buf, 100);

    nrf_delay_ms(1000);

    //Record is readable now
    NRF_LOG_INFO("Record should be readable");
    record_read_file(fileID, fileKey);

    nrf_delay_ms(1000);


    //Delete Record
    NRF_LOG_INFO("Deleting Record");
    record_delete(fileID, fileKey);
    fds_garbage_collection();

    nrf_delay_ms(1000);

    //Records are empty (Nothing at File 1, Key 1)
    NRF_LOG_INFO("Record should be empty");
    record_read_file(fileID, fileKey);

}