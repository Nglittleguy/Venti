#include "venti.h"

//////////////////////////////////////////////////////////////////////////////////
//Flash Storage Services

static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        ret_code_t err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}

ret_code_t resetFlash(nrf_fstorage_t * fstorage, uint32_t addr) {
    NRF_LOG_INFO("Clearing flash page.");
    ret_code_t err = nrf_fstorage_erase(fstorage, addr, 1, NULL);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_erase() returned: %s\n", nrf_strerror_get(err));
    }
    return err;
}

ret_code_t writeFlash(nrf_fstorage_t * fstorage, uint32_t addr, void* data, uint32_t len) {
    NRF_LOG_INFO("Request Flash Write...");
    wait_for_flash_ready(fstorage);
    NRF_LOG_INFO("Ready.");

    ret_code_t err = nrf_fstorage_write(fstorage, addr, data, len, NULL);
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_erase() returned: %s\n", nrf_strerror_get(err));
    }
    return err;
}

ret_code_t readFlash(nrf_fstorage_t * fstorage, uint32_t addr, uint8_t* data_buffer, uint32_t len){
    NRF_LOG_INFO("Request Flash Read...");

    wait_for_flash_ready(fstorage);

    NRF_LOG_INFO("Ready.");
    
    NRF_LOG_INFO("Reading from flash.");
    /* Read data. */
    ret_code_t err = nrf_fstorage_read(fstorage, addr, data_buffer, len);
 
    if (err != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n",nrf_strerror_get(err));
        return;
    }
    return err;
}

//////////////////////////////////////////////////////////////////////////////////
//Motor Services
bool lights = false;

void flipLights(bool turnOn) {
  if(lights != turnOn) {
    for(int i = 1; i<4; i++) {
      bsp_board_led_invert(i);
    }
    lights = turnOn;
  }
}

void bsp_board_motor_init(void) {
    for(int i = 0; i<4; i++) {
        nrf_gpio_cfg_output(MOTORBASEPIN+i);
    }
}

void reset_motor() {
    //Set the motor to the same start state
    for(int pinI = 0; pinI < 4; pinI++) {
        if(motor_reset[pinI])
           nrf_gpio_pin_set(MOTORBASEPIN+pinI);
        else 
           nrf_gpio_pin_clear(MOTORBASEPIN+pinI);
    }
}

void rotateCW() {
    reset_motor();
    for(int step = 0; step<STEPCOUNT; step++) {
        for(int pinI = 0; pinI < 4; pinI++) {
            if(cw_seq[step%4][pinI])
                nrf_gpio_pin_set(MOTORBASEPIN+pinI);
            else 
                nrf_gpio_pin_clear(MOTORBASEPIN+pinI);
        }
        nrf_delay_ms(STEPDELAY);
    }
}

void rotateCCW() {
    reset_motor();
    for(int step = 0; step<STEPCOUNT; step++) {

    //The CCW pattern must be set backwards... for some reason
        for(int pinI = 3; pinI > 0; pinI--) {
            if(ccw_seq[step%4][pinI])
                nrf_gpio_pin_set(MOTORBASEPIN+pinI);
            else 
                nrf_gpio_pin_clear(MOTORBASEPIN+pinI);
        }
        nrf_delay_ms(STEPDELAY);
    }
}

