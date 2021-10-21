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
/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_delay.h"
#include "boards.h"

static bool cw_seq[8][4] = {
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1},
  };

static bool ccw_seq[8][4] = {
    {1,0,0,1},
    {0,0,0,1},
    {0,0,1,1},
    {0,0,1,0},
    {0,1,1,0},
    {0,1,0,0},
    {1,1,0,0},
    {1,0,0,0},
  };

#define STEPCOUNT 2048
#define STEPDELAY 10

static void bsp_board_motor_init(void) {
    for(int i = 0; i<4; i++) {
        nrf_gpio_cfg_output(11+i);
    }
}

static void rotateCW() {
    for(int step = 0; step<STEPCOUNT; step++) {
        
        for(int pinI = 0; pinI < 4; pinI++) {
            if(cw_seq[step%8][pinI])
                nrf_gpio_pin_set(11+pinI);
            else 
                nrf_gpio_pin_clear(11+pinI);
        }
        //NRF_LOG_DEBUG("%d%d%d%d\n", cw_seq[step%8][0], cw_seq[step%8][1], cw_seq[step%8][2], cw_seq[step%8][3]);
        nrf_delay_ms(STEPDELAY);
    }
}

static void rotateCCW() {
    for(int step = 0; step<STEPCOUNT; step++) {
        
        for(int pinI = 0; pinI < 4; pinI++) {

            if(ccw_seq[step%8][pinI])
                nrf_gpio_pin_set(11+pinI);
            else 
                nrf_gpio_pin_clear(11+pinI);
        }
        //NRF_LOG_DEBUG("%d%d%d%d\n", ccw_seq[step%8][0], ccw_seq[step%8][1], ccw_seq[step%8][2], ccw_seq[step%8][3]);
        nrf_delay_ms(STEPDELAY);
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /* Configure board. */
    //bsp_board_init(BSP_INIT_LEDS);
    bsp_board_motor_init();
    /* Toggle LEDs. */
    //while (true)
    //{
        //for (int i = LEDS_NUMBER; i >= 0; i--)
        //{
        //    bsp_board_led_invert(i);
        //    nrf_delay_ms(200);
        //}

        rotateCCW();
    //    nrf_delay_ms(1000);
    //    rotateCW();
    //    nrf_delay_ms(1000);
    //}
}

/**
 *@}
 **/
