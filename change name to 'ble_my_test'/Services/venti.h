#ifndef VENTI_H
#define VENTI_H

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
#define LED_BLINK_INTERVAL              APP_TIMER_TICKS(5000)


//////////////////////////////////////////////////////////////////////////////////
//Flash Storage Services
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage);
void print_flash_info(nrf_fstorage_t * p_fstorage);
ret_code_t resetFlash(nrf_fstorage_t * fstorage, uint32_t addr);
ret_code_t writeFlash(nrf_fstorage_t * fstorage, uint32_t addr, void* data, uint32_t len);
ret_code_t readFlash(nrf_fstorage_t * fstorage, uint32_t addr, uint8_t* data_buffer, uint32_t len);

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

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


/////////////////////////////////////////////////////////////////////////////////
//Motor Services

#define STEPCOUNT 1024
#define STEPDELAY 10
#define MOTORBASEPIN 11

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

void rotateCW();

void rotateCCW();

#endif