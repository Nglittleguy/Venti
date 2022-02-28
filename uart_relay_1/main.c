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


static uint16_t m_conn_handle;

//****************************************** Mine
#include "venti.h"

////APP_TIMER_DEF(m_battery_timer_id);
////APP_TIMER_DEF(m_led_blink_id);
//APP_TIMER_DEF(m_time_segment_id);


////static uint32_t current_epoch_sec = 1644351445;
////static uint16_t current_time_segment = 0;     //Mon 12am


////uint16_t dataFileID = 0;                      //Can't be 0xFFFF
////uint16_t dataFileRecord = 1;                  //Can't be 0x0000

////bool to_write_voltage = false;

//uint8_t data_read[32] = {0};
////uint8_t data_cmp[32] = {0};
////uint8_t data_flash[32] = {0};
////uint8_t data_flash_send_to_phone[768] = {0};

//////Buffer to write to flash (asynchronous) must not be on stack
////static char flash_write_buf[32];
////static char flash_write_temp[32];

////static void wait_for_fds_ready() {
////    while(!m_fds_initialized) {
////        sd_app_evt_wait();
////    }
////}

////static void record_day_voltage();

////static void fds_evt_handler(fds_evt_t const * p_evt)
////{
////    if (p_evt->result == NRF_SUCCESS)
////    {
////        NRF_LOG_INFO("Event: %s received (NRF_SUCCESS)",
////                      fds_evt_str[p_evt->id]);
////    }
////    else
////    {
////        NRF_LOG_INFO("Event: %s received (%s)",
////                      fds_evt_str[p_evt->id],
////                      fds_err_str(p_evt->result));
////    }

////    switch (p_evt->id)
////    {
////        case FDS_EVT_INIT:
////            if (p_evt->result == NRF_SUCCESS)
////            {
////                m_fds_initialized = true;
////            }
////            break;

////        case FDS_EVT_WRITE:
////        {
////            if (p_evt->result == NRF_SUCCESS)
////            {
////                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
////                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
////                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
////            }
////            if(to_write_voltage) {
////                record_day_voltage();
////            }
////        } break;

////        case FDS_EVT_DEL_RECORD:
////        {
////            if (p_evt->result == NRF_SUCCESS)
////            {
////                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
////                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
////                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
////            }
////            m_delete_all.pending = false;
////        } break;

////        default:
////            break;
////    }
////}



////static void flash_storage_init() {
////    fds_stat_t stat = {0};
////    fds_record_desc_t desc = {0};
////    fds_find_token_t  tok  = {0};

////    (void) fds_register(fds_evt_handler);
////    ret_code_t err = fds_init();
////    APP_ERROR_CHECK(err);

////    wait_for_fds_ready();
////    err= fds_stat(&stat);
////    APP_ERROR_CHECK(err);

////    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
////    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

////    err = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);
////    if(err==NRF_SUCCESS) {
////        /* A config file is in flash. Let's update it. */
////        fds_flash_record_t config = {0};

////        /* Open the record and read its contents. */
////        err = fds_record_open(&desc, &config);
////        APP_ERROR_CHECK(err);

////        /* Copy the configuration from flash into m_dummy_cfg. */
////        memcpy(&m_dummy_cfg, config.p_data, sizeof(configuration_t));

////        NRF_LOG_INFO("Config file found, updating boot count to %d.", m_dummy_cfg.boot_count);

////        /* Update boot count. */
////        m_dummy_cfg.boot_count++;

////        /* Close the record when done reading. */
////        err = fds_record_close(&desc);
////        APP_ERROR_CHECK(err);

////        /* Write the updated record to flash. */
////        err = fds_record_update(&desc, &m_dummy_record);
////        if ((err != NRF_SUCCESS) && (err == FDS_ERR_NO_SPACE_IN_FLASH))
////        {
////            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
////        }
////        else
////        {
////            APP_ERROR_CHECK(err);
////        }
////    }
////    else {
////        /* System config not found; write a new one. */
////        NRF_LOG_INFO("Writing config file...");

////        err = fds_record_write(&desc, &m_dummy_record);
////        if ((err != NRF_SUCCESS) && (err == FDS_ERR_NO_SPACE_IN_FLASH))
////        {
////            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
////        }
////        else
////        {
////            APP_ERROR_CHECK(err);
////        }
////    }
////}


////static void record_read_file(uint16_t fileID, uint16_t recordKey) {
////    NRF_LOG_INFO("Trying to read");
////    fds_find_token_t tok   = {0};
////    fds_record_desc_t desc = {0};
////    fds_flash_record_t rec;

////    uint16_t bufferIndex = 0, i = 0;
////    memset(data_flash_send_to_phone, 0, 768);

////    while(fds_record_find(fileID, recordKey, &desc, &tok)==NRF_SUCCESS) {
////        if(fds_record_open(&desc, &rec)!=NRF_SUCCESS) {
////            NRF_LOG_INFO("Error in opening record");
////        }
        
////        uint8_t* raw_data = (uint8_t *)rec.p_data;
////        printf("Record: %s\n\r", raw_data);

////        i = 0;
////        while(raw_data[i]!='\0') {
////            data_flash_send_to_phone[bufferIndex] = raw_data[i];
////            i++;
////            bufferIndex++;
////        }
////        data_flash_send_to_phone[bufferIndex] = '\n';
////        bufferIndex++;


////        if(fds_record_close(&desc)!=NRF_SUCCESS) {
////            NRF_LOG_INFO("Error in closing record");
////        }
////    }
////    printf("Sent: %s\n\r", data_flash_send_to_phone);

////    ret_code_t err_code = ble_nus_data_send(&m_nus, data_flash_send_to_phone, &bufferIndex, m_conn_handle);
////    if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
////    {
////        APP_ERROR_CHECK(err_code);
////    }
////}

////static void record_write(uint32_t fid, uint32_t key, void const * p_data, uint32_t len)
////{
////    fds_record_t const rec =
////    {
////        .file_id           = fid,
////        .key               = key,
////        .data.p_data       = p_data,
////        .data.length_words = (len + 3) / sizeof(uint32_t)
////    };

////    NRF_LOG_INFO("writing record to flash...\nfile: 0x%x, key: 0x%x, \"%s\", len: %u bytes\n", fid, key, p_data, len);

////    ret_code_t err = fds_record_write(NULL, &rec);
////    if (err != NRF_SUCCESS)
////    {
////        NRF_LOG_INFO("error: fds_record_write() returned %s.\n", fds_err_str(err));
////    }
////}

////static void record_day_voltage() {
////    dataFileID++;
////    setDayVoltageBuffer(flash_write_buf, current_epoch_sec);
////    record_write(dataFileID, 0x0001, &flash_write_buf, 32);
////}

////static void record_temp() {
////    setTemperatureBuffer(flash_write_temp, current_time_segment);
////    record_write(dataFileID, 0x0002, &flash_write_temp, 32);
////}

////static void print_all_cmd()
////{
////    fds_find_token_t tok   = {0};
////    fds_record_desc_t desc = {0};

////    while (fds_record_iterate(&desc, &tok) != FDS_ERR_NOT_FOUND)
////    {
////        ret_code_t err;
////        fds_flash_record_t frec = {0};

////        err = fds_record_open(&desc, &frec);
////        switch (err)
////        {
////            case NRF_SUCCESS:
////                break;

////            case FDS_ERR_CRC_CHECK_FAILED:
////                printf("error: CRC check failed!\n");
////                continue;

////            case FDS_ERR_NOT_FOUND:
////                printf("error: record not found!\n");
////                continue;

////            default:
////            {
////                printf("error: unexpecte error %s.\n",fds_err_str(err));
////                continue;
////            }
////        }

////        uint32_t const len = frec.p_header->length_words * sizeof(uint32_t);

////        printf(" 0x%04x\t"
////                        "\t 0x%04x\t"
////                        "\t 0x%04x\t"
////                        "\t %4u bytes\n",
////                        frec.p_header->record_id,
////                        frec.p_header->file_id,
////                        frec.p_header->record_key,
////                        len);

////        err = fds_record_close(&desc);
////        APP_ERROR_CHECK(err);
////    }
////}

////******************************************



#define DEVICE_NAME                     "Nordic Relay 32"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                0 //Continue advertising (18000 default)        /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

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
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */


BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

NRF_BLE_GQ_DEF(m_ble_gatt_queue, NRF_SDH_BLE_CENTRAL_LINK_COUNT, NRF_BLE_GQ_QUEUE_SIZE);   /**< BLE GATT Queue instance. */
   


//static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */



///* YOUR_JOB: Declare all services structure your application is using
// *  BLE_XYZ_DEF(m_xyz);
// */

//// YOUR_JOB: Use UUIDs for service(s) used in your application.

//static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
//{
//    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
//};

////static ble_uuid_t m_adv_uuids[] =  
////{
////    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
////};


//static void advertising_start();


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

//static void battery_level_update(void) 
//{
//    ret_code_t err_code;

//    uint8_t   battery_level;
//    uint16_t  vbatt;
//    battery_voltage_get(&vbatt);

//    battery_level = battery_level_in_percent(vbatt);
//    printf("ADC result %d, in percent: %d\r\n", vbatt, battery_level);

//    err_code = ble_bas_battery_level_update(&m_bas, battery_level, m_conn_handle);
//    if((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
//        APP_ERROR_HANDLER(err_code);
//}


//static void battery_level_meas_timeout_handler(void * p_context)
//{
//    UNUSED_PARAMETER(p_context);
//    printf("Battery Level Timeout Event");

//    if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
//        battery_level_update();
//}

///**@brief Function for handling LED write events
// * @param[in] p_led_service   Instance of LED Service to which the write applies
// * @param[in] led_state       Written/desired state of LED
// */
//static void led_write_handler(uint16_t conn_hanlde, ble_led_service_t * p_led_service, uint8_t led_state) 
//{
//    if(led_state)
//    {
//        bsp_board_led_on(LIGHTBULB_LED);
//        NRF_LOG_INFO("Received LED on!\n");
//    }
//    else 
//    {
//        bsp_board_led_off(LIGHTBULB_LED);
//        NRF_LOG_INFO("Recieved LED off!\n");
//    }
//}

//static void led_blink_timeout_handler(void * p_context)
//{
//    UNUSED_PARAMETER(p_context);
//    //NRF_LOG_INFO("LED Blink\n");
//    //bsp_board_led_on(BLINK_LED);
//    //nrf_delay_ms(1000);
//    //bsp_board_led_off(BLINK_LED);
//    //float current_temp = ds18b20_read_temp();
//    //printf("Temp: %f\n\r", current_temp);
    
//}

////static void time_segment_timeout_handler(void *p_context)
////{
////    UNUSED_PARAMETER(p_context);
////    current_time_segment++;

////    //New Hour 
////    if(current_time_segment % 12 == 0) {

////        //New Day
////        if(current_time_segment % 288 == 0) {
////            current_epoch_sec += 86400;

////            //New Week, reset to 0
////            if(current_time_segment == 2016) {
////                current_time_segment = 0;
////            } 

////            to_write_voltage = true;
////        }

////        //Write to file temp min, max
////        record_temp();
////        resetTemperatureMinMax();
////    }
////    else {
////        compareTemperatureMinMax();
////    }

////    if(checkSchedule(current_time_segment)) {
////        NRF_LOG_INFO("Schedule rotated the vent");
////    }




////}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    //err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
    //APP_ERROR_CHECK(err_code);

    //err_code = app_timer_create(&m_led_blink_id, APP_TIMER_MODE_REPEATED, led_blink_timeout_handler);
    //APP_ERROR_CHECK(err_code);

    //err_code = app_timer_create(&m_time_segment_id, APP_TIMER_MODE_REPEATED, time_segment_timeout_handler);
    //APP_ERROR_CHECK(err_code);

    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       ret_code_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
}


/////**@brief Function for the GAP initialization.
//// *
//// * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
//// *          device including the device name, appearance, and the preferred connection parameters.
//// */
////static void gap_params_init(void)
////{
////    ret_code_t              err_code;
////    ble_gap_conn_params_t   gap_conn_params;
////    ble_gap_conn_sec_mode_t sec_mode;

////    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

////    err_code = sd_ble_gap_device_name_set(&sec_mode,
////                                          (const uint8_t *)DEVICE_NAME,
////                                          strlen(DEVICE_NAME));
////    APP_ERROR_CHECK(err_code);

////    /* YOUR_JOB: Use an appearance value matching the application's use case.
////       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
////       APP_ERROR_CHECK(err_code); */

////    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

////    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
////    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
////    gap_conn_params.slave_latency     = SLAVE_LATENCY;
////    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

////    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
////    APP_ERROR_CHECK(err_code);
////}



/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
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


///**@brief Function for handling Queued Write Module errors.
// *
// * @details A pointer to this function will be passed to each service which may need to inform the
// *          application about an error.
// *
// * @param[in]   nrf_error   Error code containing information about what went wrong.
// */
//static void nrf_qwr_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}


///**@brief   Function for handling app_uart events.
// *
// * @details This function will receive a single character from the app_uart module and append it to
// *          a string. The string will be be sent over BLE when the last character received was a
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
// */
///**@snippet [Handling the data received over UART] */
//void uart_event_handle(app_uart_evt_t * p_event)
//{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint8_t index = 0;
//    uint32_t       err_code;

//    switch (p_event->evt_type)
//    {
//        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;

//            if ((data_array[index - 1] == '\n') ||
//                (data_array[index - 1] == '\r') ||
//                (index >= m_ble_nus_max_data_len))
//            {
//                if (index > 1)
//                {
//                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
//                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                    
//                    do
//                    {
//                        uint16_t length = (uint16_t)index;
//                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
//                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
//                            (err_code != NRF_ERROR_RESOURCES) &&
//                            (err_code != NRF_ERROR_NOT_FOUND))
//                        {
//                            APP_ERROR_CHECK(err_code);
//                        }
//                    } while (err_code == NRF_ERROR_RESOURCES);
//                }

//                index = 0;
//            }
//            break;

//        case APP_UART_COMMUNICATION_ERROR:
//            NRF_LOG_ERROR("Communication error occurred while handling UART.");
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;

//        case APP_UART_FIFO_ERROR:
//            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;

//        default:
//            break;
//    }
//}
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

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                do
                {
                    ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
                    if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
                    {
                        APP_ERROR_CHECK(ret_val);
                    }
                } while (ret_val == NRF_ERROR_RESOURCES);

                index = 0;
            }
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
    
    //UART PINS CHECK (don't actually need)
    NRF_UART0->TASKS_STOPRX=1;
    NRF_UART0->TASKS_STOPTX=1;
}
/**@snippet [UART Initialization] */


///**@brief Function for handling the data from the Nordic UART Service.
// *
// * @details This function will process the data received from the Nordic UART BLE Service and send
// *          it to the UART module.
// *
// * @param[in] p_evt       Nordic UART Service event.
// */
///**@snippet [Handling the data received over BLE] */
//static void nus_data_handler(ble_nus_evt_t * p_evt)
//{
//    //TEST - check the 'conn_handle' of the p_evt to get which connection it is coming from
//    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
//    {
//        //uint32_t err_code;
//        //NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
//        //NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

//        //memset(data_read, '\0', 32);
//        //for (uint32_t i = 0; i < p_evt->params.rx_data.length && i < 32; i++) {
//        //    data_read[i] = p_evt->params.rx_data.p_data[i];
//        //}

//        //if(strcmp(data_read, "on")==0) {
//        //    flipLights(true);
//        //}
//        //else if(strcmp(data_read, "off")==0) {
//        //    flipLights(false);
//        //}
//        //else if(strcmp(data_read, "left")==0) {
//        //    NRF_LOG_INFO("left");
//        //    rotateCCWHalf();
//        //}
//        //else if(strcmp(data_read, "right")==0) {
//        //    NRF_LOG_INFO("right");
//        //    rotateCWHalf();
//        //}
//        ////else if(strcmp(data_read, "read")==0) {
//        ////    record_read();
//        ////}
//        //else if(strcmp(data_read, "write")==0) {
//        //    record_day_voltage();
//        //}

//        ////Open The vent to an amount
//        //else if(strncmp(data_read, "open", 4)==0) {
//        //    uint8_t target = (uint8_t) atoi(data_read+5);
//        //    printf("Rotate to %d", target);
//        //    rotate(target);
//        //}

//        ////Read the current temperature
//        //else if(strcmp(data_read, "temp")==0) {
//        //    char data_array[10];
//        //    uint16_t length = (uint16_t)10;
//        //    float current_temp = ds18b20_read_temp();
//        //    sprintf(data_array, "%fC", current_temp);
//        //    printf("Read temperature as: %f\n", current_temp);
//        //    err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
//        //    if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
//        //    {
//        //        APP_ERROR_CHECK(err_code);
//        //    }
//        //}
        
//        ////Set a schedule
//        //else if(strncmp(data_read, "schedule", 8)==0) {
//        //    char *ptr = strtok(data_read, " ");

//        //    ptr = strtok(NULL, " ");
//        //    uint8_t slot = atoi(ptr);
           
//        //    ptr = strtok(NULL, " ");
//        //    uint16_t time = atoi(ptr);

//        //    ptr = strtok(NULL, " ");
//        //    uint16_t amount = atoi(ptr);

//        //    printf("Received Scheduling: %d, %d, %d\n\r", slot, time, amount);
//        //    addToSchedule(slot, time, amount);
//        //    printSchedule();
//        //}

//        //else if(strncmp(data_read, "read", 4)==0) {
//        //    char *ptr = strtok(data_read, " ");

//        //    ptr = strtok(NULL, " ");
//        //    uint8_t fileID = atoi(ptr);
           
//        //    ptr = strtok(NULL, " ");
//        //    uint16_t recordKey = atoi(ptr);

//        //    printf("Received Read of File: %d, %d\n\r", fileID, recordKey);
//        //    record_read_file(fileID, recordKey);
//        //}

//        //Don't actually need to output to UART, just read in
//        /*
//        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
//        {
//            do
//            {
//                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
//                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
//                {
//                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
//                    APP_ERROR_CHECK(err_code);
//                }
//            } while (err_code == NRF_ERROR_BUSY);
//        }
//        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
//        {
//            while (app_uart_put('\n') == NRF_ERROR_BUSY);
//        }
//        */
//    }

//}
///**@snippet [Handling the data received over BLE] */


///**@brief Function for handling the YYY Service events.
// * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
// *
// * @details This function will be called for all YY Service events which are passed to
// *          the application.
// *
// * @param[in]   p_yy_service   YY Service structure.
// * @param[in]   p_evt          Event received from the YY Service.
// *
// *
//static void on_yys_evt(ble_yy_service_t     * p_yy_service,
//                       ble_yy_service_evt_t * p_evt)
//{
//    switch (p_evt->evt_type)
//    {
//        case BLE_YY_NAME_EVT_WRITE:
//            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
//            break;

//        default:
//            // No implementation needed.
//            break;
//    }
//}
//*/

///**@brief Function for initializing services that will be used by the application.
// */
//static void services_init(void)
//{
//    uint32_t         err_code;

    
//    ble_bas_init_t          bas_init;
//    ble_led_service_init_t  led_init;
    
//    ble_nus_init_t     nus_init;

//    nrf_ble_qwr_init_t qwr_init = {0};

//    // Initialize Queued Write Module.
//    qwr_init.error_handler = nrf_qwr_error_handler;

//    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
//    APP_ERROR_CHECK(err_code);

    
//    //Initialize LED service
//    led_init.led_write_handler = led_write_handler;
//    err_code = ble_led_service_init(&m_led_service, &led_init);
//    APP_ERROR_CHECK(err_code);


//    //Initialize Battery Service
//    memset(&bas_init, 0, sizeof(bas_init));
//    //Battery Service can be changed/increased in security level
//    bas_init.bl_rd_sec = 1;
//    bas_init.bl_cccd_wr_sec = 1;
//    bas_init.bl_report_rd_sec = 1;
    

//    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.bl_cccd_wr_se);
//    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
//    //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);
//    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    
//    bas_init.evt_handler    = NULL;
//    bas_init.support_notification = true;
//    bas_init.p_report_ref   = NULL;
//    bas_init.initial_batt_level = 100;

//    err_code = ble_bas_init(&m_bas, &bas_init);
//    APP_ERROR_CHECK(err_code);
    

//    //Initialize UART/NUS
//    memset(&nus_init, 0, sizeof(nus_init));
//    nus_init.data_handler = nus_data_handler;
//    err_code = ble_nus_init(&m_nus, &nus_init);
//    APP_ERROR_CHECK(err_code);

//    initSchedule();

//    /* YOUR_JOB: Add code to initialize the services used by the application.
//       ble_xxs_init_t                     xxs_init;
//       ble_yys_init_t                     yys_init;

//       // Initialize XXX Service.
//       memset(&xxs_init, 0, sizeof(xxs_init));

//       xxs_init.evt_handler                = NULL;
//       xxs_init.is_xxx_notify_supported    = true;
//       xxs_init.ble_xx_initial_value.level = 100;

//       err_code = ble_bas_init(&m_xxs, &xxs_init);
//       APP_ERROR_CHECK(err_code);

//       // Initialize YYY Service.
//       memset(&yys_init, 0, sizeof(yys_init));
//       yys_init.evt_handler                  = on_yys_evt;
//       yys_init.ble_yy_initial_value.counter = 0;

//       err_code = ble_yy_service_init(&yys_init, &yy_init);
//       APP_ERROR_CHECK(err_code);
//     */
//}


///**@brief Function for handling the Connection Parameters Module.
// *
// * @details This function will be called for all events in the Connection Parameters Module which
// *          are passed to the application.
// *          @note All this function does is to disconnect. This could have been done by simply
// *                setting the disconnect_on_fail config parameter, but instead we use the event
// *                handler mechanism to demonstrate its use.
// *
// * @param[in] p_evt  Event received from the Connection Parameters Module.
// */
//static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
//{
//    ret_code_t err_code;

//    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
//    {
//        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
//        APP_ERROR_CHECK(err_code);
//    }
//}


///**@brief Function for handling a Connection Parameters error.
// *
// * @param[in] nrf_error  Error code containing information about what went wrong.
// */
//static void conn_params_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}


///**@brief Function for initializing the Connection Parameters module.
// */
//static void conn_params_init(void)
//{
//    ret_code_t             err_code;
//    ble_conn_params_init_t cp_init;

//    memset(&cp_init, 0, sizeof(cp_init));

//    cp_init.p_conn_params                  = NULL;
//    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
//    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
//    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
//    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
//    cp_init.disconnect_on_fail             = false;
//    cp_init.evt_handler                    = on_conn_params_evt;
//    cp_init.error_handler                  = conn_params_error_handler;

//    err_code = ble_conn_params_init(&cp_init);
//    APP_ERROR_CHECK(err_code);
//}


///**@brief Function for starting timers.
// */
//static void application_timers_start(void)
//{
//    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
//       ret_code_t err_code;
//       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
//       APP_ERROR_CHECK(err_code); */
//    uint32_t err_code;
//    //err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
//    //APP_ERROR_CHECK(err_code);

//    //err_code = app_timer_start(m_led_blink_id, LED_BLINK_INTERVAL, NULL);
//    //APP_ERROR_CHECK(err_code);

//    err_code = app_timer_start(m_time_segment_id, SEGMENT_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);
//}


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

///**@snippet [Handling events from the ble_nus_c module] */


///**
// * @brief Function for handling shutdown events.
// *
// * @param[in]   event       Shutdown type.
// */
//static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
//{
//    ret_code_t err_code;

//    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);

//    switch (event)
//    {
//        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
//            // Prepare wakeup buttons.
//            err_code = bsp_btn_ble_sleep_mode_prepare();
//            APP_ERROR_CHECK(err_code);
//            break;

//        default:
//            break;
//    }

//    return true;
//}

//NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);



/////**@brief Function for handling advertising events.
//// *
//// * @details This function will be called for advertising events which are passed to the application.
//// *
//// * @param[in] ble_adv_evt  Advertising event.
//// */
////static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
////{
////    ret_code_t err_code;

////    switch (ble_adv_evt)
////    {
////        case BLE_ADV_EVT_FAST:
////            printf("Fast advertising.");
////            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
////            APP_ERROR_CHECK(err_code);
////            break;

////        case BLE_ADV_EVT_IDLE:
////            sleep_mode_enter();
////            break;

////        default:
////            break;
////    }
////}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
   

    if(p_gap_evt->params.connected.role == BLE_GAP_ROLE_PERIPH) {
        NRF_LOG_INFO("Peripheral Role Event %d", p_gap_evt->conn_handle);
    }
    else if (p_gap_evt->params.connected.role == BLE_GAP_ROLE_CENTRAL) {
        NRF_LOG_INFO("Central Role Event %d", p_gap_evt->conn_handle);
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
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
            //printf("Connected.");
            //if (p_gap_evt->params.connected.role == BLE_GAP_ROLE_CENTRAL) {
                err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
                APP_ERROR_CHECK(err_code);

                err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
                APP_ERROR_CHECK(err_code);

                // start discovery of services. The NUS Client waits for a discovery result
                err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
                APP_ERROR_CHECK(err_code);
            //}
            //else {
            //    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //    APP_ERROR_CHECK(err_code);
            //    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            //    APP_ERROR_CHECK(err_code);
            //}
            
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

        //case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        //    // No system attributes have been stored.
        //    NRF_LOG_DEBUG("GATT Event System Attribute Missing.");
        //    err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
        //    APP_ERROR_CHECK(err_code);
        //    break;

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



/////**@brief Clear bond information from persistent storage.
//// */
////static void delete_bonds(void)
////{
////    ret_code_t err_code;

////    printf("Erase bonds!");

////    err_code = pm_peers_delete();
////    APP_ERROR_CHECK(err_code);
////}


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
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        //case BSP_EVENT_WHITELIST_OFF:
        //    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
        //    {
        //        err_code = ble_advertising_restart_without_whitelist(&m_advertising);
        //        if (err_code != NRF_ERROR_INVALID_STATE)
        //        {
        //            APP_ERROR_CHECK(err_code);
        //        }
        //    }
        //    break; // BSP_EVENT_KEY_0
     
        //case BSP_EVENT_KEY_1:
        //    NRF_LOG_INFO("2 pressed\n");
        //    do {
        //      uint8_t data_array[10] = "2 pressed\n";
        //      uint16_t length = (uint16_t)10;
        //      err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
        //      if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
        //      {
        //        APP_ERROR_CHECK(err_code);
        //      }
        //   } while (err_code == NRF_ERROR_RESOURCES);
        //break;                  
        //case BSP_EVENT_KEY_2:
        // NRF_LOG_INFO("3 pressed\n");
        //    do {
        //      uint8_t data_array[10] = "3 pressed\n";
        //      uint16_t length = (uint16_t)10;
        //      err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
        //      if ((err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) && (err_code != NRF_ERROR_NOT_FOUND))
        //      {
        //        APP_ERROR_CHECK(err_code);
        //      }
        //    } while (err_code == NRF_ERROR_RESOURCES);
        //break;
        //case BSP_EVENT_KEY_3:
        // NRF_LOG_INFO("4 pressed\n");
        //    do {
        //      uint8_t data_array[10] = "4 pressed\n";
        //      uint16_t length = (uint16_t)10;
        //      err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
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


/////**@brief Function for initializing the Advertising functionality.
//// */
////static void advertising_init(void)
////{
////    ret_code_t             err_code;
////    ble_advertising_init_t init;

////    memset(&init, 0, sizeof(init));

////    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
////    init.advdata.include_appearance      = true;
////    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
////    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
////    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

////    init.config.ble_adv_fast_enabled  = true;
////    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
////    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
////    init.evt_handler = on_adv_evt;

////    err_code = ble_advertising_init(&m_advertising, &init);
////    APP_ERROR_CHECK(err_code);

////    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
////}

static void leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    ret_code_t err_code;
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
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


///**@brief Function for starting advertising.
// */
//static void advertising_start()
//{
//    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
//    APP_ERROR_CHECK(err_code);
//}

////******************************************
//// Central NUS Services




   
//typedef void (*app_nus_client_on_data_received_t)(const uint8_t *data_ptr, uint16_t data_length);
         
//static app_nus_client_on_data_received_t m_on_data_received = 0;

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

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len-1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY);
    }
    //if (ECHOBACK_BLE_UART_DATA)
    //{
    //    // Send data back to the peripheral.
    //    do
    //    {
    //        ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
    //        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
    //        {
    //            NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
    //            APP_ERROR_CHECK(ret_val);
    //        }
    //    } while (ret_val == NRF_ERROR_BUSY);
    //}
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
    NRF_LOG_INFO("Scan Event Occurred");
    
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
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
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


/**@brief Function for initializing the scanning and setting the filters.
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


    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_UUID_FILTER, false);
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
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            //if(m_on_data_received)
            //{
            //    NRF_LOG_INFO("Receiving Information From Relay");
            //    m_on_data_received(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
            //}
            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
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
    bool erase_bonds;

    
    // Initialize.
    log_init();
    timers_init();
//    //battery_voltage_init();
    leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
//    //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
//    //gap_params_init();
    gatt_init();
    uart_init();
//    services_init();
//    //advertising_init();
//    conn_params_init();
//    //application_timers_start();
//    bsp_board_motor_init();
//    //flash_storage_init();
//    //print_all_cmd();
    
//    //record_day_voltage();

//    // Start execution.
    printf("My Test App Started.");
//    //application_timers_start();

//    //advertising_start();
    app_nus_client_init();
    
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();;
    }
}


/**
 * @}
 */


///**
// * @brief Function for handling shutdown events.
// *
// * @param[in]   event       Shutdown type.
// */
//static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
//{
//    ret_code_t err_code;

//    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);

//    switch (event)
//    {
//        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
//            // Prepare wakeup buttons.
//            err_code = bsp_btn_ble_sleep_mode_prepare();
//            APP_ERROR_CHECK(err_code);
//            break;

//        default:
//            break;
//    }

//    return true;
//}

//NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


