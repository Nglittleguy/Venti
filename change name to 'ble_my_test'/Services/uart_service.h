#ifndef UART_SERVICE_H
#define UART_SERVICE_H

#include <stdint.h>
#include "boards.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

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
//static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
//{
//    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
//};




static void bsp_board_motor_init(void);

static void reset_motor();

static void rotateCW();

static void rotateCCW();



#endif