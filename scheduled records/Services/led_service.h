#ifndef LED_SERVICE_H
#define LED_SERVICE_H

#include <stdint.h>
#include "boards.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#define BLE_LED_SERVICE_BLE_OBSERVER_PRIO 2

#define BLE_LED_SERVICE_DEF(_name)  \
static ble_led_service_t _name;     \
NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_LED_SERVICE_BLE_OBSERVER_PRIO, ble_led_service_on_ble_evt, &_name)


//    LED service:              E54B0001-67F5-479E-8711-B3B99198CE6C
//    LED 2 characteristic:     E54B0002-67F5-479E-8711-B3B99198CE6C
//    Base UUID:                E54B0000-67F5-479E-8711-B3B99198CE6C

#define BLE_UUID_LED_SERVICE_BASE_UUID  {0x6C, 0xCE, 0x98, 0x91, 0xB9, 0xB3, 0x11, 0x87, 0x9E, 0x47, 0xF5, 0x67, 0x00, 0x00, 0x4B, 0xE5}
#define BLE_UUID_LED_SERVICE_UUID      0x001
#define BLE_UUID_LED_2_CHAR_UUID        0x002

//Custom Type of 'ble_led_service_s'
typedef struct ble_led_service_s ble_led_service_t;

typedef void (*ble_led_service_led_write_handler_t) (uint16_t conn_handle, ble_led_service_t * p_led_service, uint8_t new_state);

//All options and data needed for initialization of service
typedef struct
{
    ble_led_service_led_write_handler_t led_write_handler;
} 
ble_led_service_init_t;

//LED Service Structure
typedef struct ble_led_service_s
{
    uint16_t        conn_handle;
    uint16_t        service_handle;
    uint8_t         uuid_type;
    ble_gatts_char_handles_t            led_2_char_handles;
    ble_led_service_led_write_handler_t   led_write_handler;
}
ble_led_service_t;

//Function to Initialize LED Services
uint32_t ble_led_service_init(ble_led_service_t * p_led_service, const ble_led_service_init_t * p_led_service_init);

//Function for handling application BLE stack events
void ble_led_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#endif

