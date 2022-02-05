#include "venti.h"

//////////////////////////////////////////////////////////////////////////////////
//Flash Storage Services



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

void rotateCWHalf() {
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

void rotateCCWHalf() {
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


static uint8_t current_open_amount = 255;


void rotateCW(short amount) {
    reset_motor();
    for(int step = 0; step<(amount<<1); step++) {
        for(int pinI = 0; pinI < 4; pinI++) {
            if(cw_seq[step%4][pinI])
                nrf_gpio_pin_set(MOTORBASEPIN+pinI);
            else 
                nrf_gpio_pin_clear(MOTORBASEPIN+pinI);
        }
        nrf_delay_ms(STEPDELAY);
    }
}

void rotateCCW(short amount) {
    reset_motor();
    for(int step = 0; step<(amount<<1); step++) {

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

void rotate(uint8_t target_open_amount) {
    printf("What's going on? %d\n", target_open_amount);
    
    short steps_to_take = current_open_amount - target_open_amount;
    current_open_amount = target_open_amount;
    if(steps_to_take < 0) {
        printf("CCW\n");
        rotateCCW(-steps_to_take);
    }
    else {
        printf("CW\n");
        rotateCW(steps_to_take);
    }
}

