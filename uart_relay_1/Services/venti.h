#ifndef VENTI_H
#define VENTI_H

//////////////////////////////////////////////////////////////////////////////////
//Relaying Services

#define DEVICE_ID     1                 /**< ID of Device for Relaying. */

#if DEVICE_ID == 1
#define DEVICE_NAME   "S-Vent 1"        /**< Name of device. Will be included in the advertising data. */
#define RIGHT_NAME    "My Nordic_UART"        /**< Name of device to connect to via Central. */
#endif

#if DEVICE_ID == 2
#define DEVICE_NAME   "S-Vent 2"        /**< Name of device. Will be included in the advertising data. */
#define RIGHT_NAME    "S-Vent 1"        /**< Name of device to connect to via Central. */
#endif

#if DEVICE_ID == 3
#define DEVICE_NAME   "S-Vent 3"        /**< Name of device. Will be included in the advertising data. */
#define RIGHT_NAME    "S-Vent 2"        /**< Name of device to connect to via Central. */
#endif

#if DEVICE_ID == 4
#define DEVICE_NAME   "S-Vent 4"        /**< Name of device. Will be included in the advertising data. */
#define RIGHT_NAME    "S-Vent 3"        /**< Name of device to connect to via Central. */
#endif


BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */


//////////////////////////////////////////////////////////////////////////////////
//Battery & Blinking LED Services

#include "ble_bas.h"
#include "led_service.h"
#include "Battery Level/battery_voltage.h"

#define LIGHTBULB_LED                   BSP_BOARD_LED_1
BLE_BAS_DEF(m_bas);
BLE_LED_SERVICE_DEF(m_led_service);

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(300000)       //120000
#define BLINK_LED                       BSP_BOARD_LED_3
#define LED_BLINK_INTERVAL              APP_TIMER_TICKS(1000)


//////////////////////////////////////////////////////////////////////////////////
//Flash Storage Services
#include "fds.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;


#define CONFIG_FILE     (0x8010)
#define CONFIG_REC_KEY  (0x7010)

/* A dummy structure to save in flash. */
typedef struct
{
    uint32_t boot_count;
    char     device_name[16];
    bool     config1_on;
    bool     config2_on;
} configuration_t;

/* Dummy configuration data. */
static configuration_t m_dummy_cfg =
{
    .config1_on  = false,
    .config2_on  = true,
    .boot_count  = 0x0,
    .device_name = "dummy",
};

/* A record containing dummy configuration data. */
static fds_record_t const m_dummy_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &m_dummy_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_dummy_cfg) + 3) / sizeof(uint32_t),
};

void delete_all_begin(void);
bool record_delete_next(void);
void delete_all_process(void);

const char *fds_err_str(ret_code_t ret);

extern bool low_battery;

void setDayVoltageBuffer(char* flash_write_buf, uint32_t epoch);
void setTemperatureBuffer(char* flash_write_temp, uint16_t segment);

/////////////////////////////////////////////////////////////////////////////////
//UART (NUS) Services

#include "nrf_delay.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

//static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */



/////////////////////////////////////////////////////////////////////////////////
//Motor Services

#define STEPCOUNT 1024
#define STEPDELAY 10
#define MOTORBASEPIN 11             //Deprecated, use motor_pins instead

static const uint8_t motor_pins[4] = {11, 12, 13, 14};

static const bool motor_reset[4] = {1,0,0,1};

static const bool cw_seq[4][4] = {
    {1, 0, 0, 1},
    {1, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 1},
};

static const bool ccw_seq[4][4] = {
    {1, 0, 0, 1},
    {0, 0, 1, 1},
    {0, 1, 1, 0},
    {1, 1, 0, 0}
};

void flipLights(bool turnOn);

void bsp_board_motor_init(void);

void reset_motor();

void zeroMotor();

void rotateCWHalf();

void rotateCCWHalf();

void rotateCW(short amount);

void rotateCCW(short amount);

void rotate(uint8_t val);

uint8_t getOpenAmount();

#endif

/////////////////////////////////////////////////////////////////////////////////
//Temperature Sensor - https://github.com/DSysoletin/nrf52_ds18b20_example/blob/master/main.c

#define DS18B20PIN 16

void ds18b20_send(char bit);
unsigned char ds18b20_read(void);
void ds18b20_send_byte(char data);
unsigned char ds18b20_read_byte(void);
bool ds18b20_reset_and_check(void);
float ds18b20_read_temp(void);

extern short temp_max;
extern short temp_min;

/////////////////////////////////////////////////////////////////////////////////
//Scheduling System

typedef struct
{
    uint16_t time;
    uint8_t amount;

}Schedule_event;

static const char daysOfWeek[7][10] = {"Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday"};

extern Schedule_event schedule[7][5];

void initSchedule();
void printSchedule();
void addToSchedule(uint8_t slot, uint16_t time, uint8_t amount);
void currentTimeFromSegment(char* buf, uint16_t time_segment);
void sendSchedule(uint8_t weekday, char* buf);
void setSchedule(uint8_t weekday, uint16_t* timeArr, uint8_t* amountArr);

/////////////////////////////////////////////////////////////////////////////////
//Timer For Events

#include "limits.h"
#define SEGMENT_INTERVAL              APP_TIMER_TICKS(10000)  //300000

void resetTemperatureMinMax();
void compareTemperatureMinMax();
bool checkSchedule(uint16_t time_segment);


