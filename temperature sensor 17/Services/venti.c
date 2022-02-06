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


/////////////////////////////////////////////////////////////////////////////////
//Temperature Sensor - https://github.com/DSysoletin/nrf52_ds18b20_example/blob/master/main.c

typedef uint8_t ScratchPad[9];

/**@brief Function for sending one bit to bus.
 */
void ds18b20_send(char bit)
{
    nrf_gpio_cfg_output(DS18B20PIN);
    nrf_gpio_pin_clear(DS18B20PIN);

    if(bit==1)
    {   
        nrf_delay_us(5);
        nrf_gpio_pin_set(DS18B20PIN);
        nrf_delay_us(65);
    }
    else {
        nrf_delay_us(60);
        nrf_gpio_pin_set(DS18B20PIN);
        nrf_delay_us(10);
    }
    
}


/**@brief Function for reading one bit from bus.
 */
unsigned char ds18b20_read(void)
{
    unsigned char presence=0;
    nrf_gpio_cfg_output(DS18B20PIN);
    nrf_gpio_pin_clear(DS18B20PIN);

    nrf_delay_us(5);

    nrf_gpio_cfg_input(DS18B20PIN,NRF_GPIO_PIN_NOPULL);

    nrf_delay_us(5);

    if(nrf_gpio_pin_read(DS18B20PIN))
    {
        presence = 1;
    }
    else
    {
        presence = 0;
    }
    
    return presence;
}


/**@brief Function for sending one byte to bus.
 */
void ds18b20_send_byte(char data)
{
    unsigned char i;
    unsigned char x;
    for(i=0;i<8;i++)
    {
      x = data>>i;
      x &= 0x01;
      ds18b20_send(x);
    }
    nrf_delay_us(200);
}

/**@brief Function for reading one byte from bus.
 */
unsigned char ds18b20_read_byte(void)
{
    unsigned char i;
    unsigned char data = 0;
    for (i=0;i<8;i++)
    {
        //if(ds18b20_read()) data|=0x01<<i;
        data = data >> 1;
        nrf_gpio_pin_clear(DS18B20PIN);
        nrf_gpio_cfg_output(DS18B20PIN);
        
        nrf_delay_us(5);

        nrf_gpio_cfg_input(DS18B20PIN,NRF_GPIO_PIN_NOPULL);

        nrf_delay_us(5);

        if(nrf_gpio_pin_read(DS18B20PIN)) {
            data |= 0x80;
        }

        nrf_delay_us(50);
    }
    
    return(data);
}

bool ds18b20_reset_and_check(void) {
    bool res=0;
    //Form reset pulse
  
    nrf_gpio_cfg_output(DS18B20PIN);
    nrf_gpio_pin_write(DS18B20PIN,0);
    nrf_delay_us(480);
    //Release bus and wait 15-60MS
    nrf_gpio_cfg_input(DS18B20PIN, NRF_GPIO_PIN_PULLUP);
    nrf_delay_us(70);

    //Read from bus
    res=nrf_gpio_pin_read(DS18B20PIN);
    while(nrf_gpio_pin_read(DS18B20PIN)!=0) {
        nrf_delay_us(10);
    }
    if(res==0)
    {
      nrf_delay_us(500);
      return true;
    }
    return false;
}

float ds18b20_read_temp(void) {
    char t1 = 0, t2 = 0;
    double f;
    ScratchPad scratchPad;
    if(ds18b20_reset_and_check()) {
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0x44);
        nrf_delay_ms(410);

        if(ds18b20_reset_and_check()) {
            ds18b20_send_byte(0xCC);
            ds18b20_send_byte(0xBE);
            for(uint8_t i = 0; i<9; i++) {
                scratchPad[i] = ds18b20_read_byte();
                printf("Scratch %d: %x\n", i, scratchPad[i]);
            }
            //t1 = ds18b20_read_byte();
            //t2 = ds18b20_read_byte();
            //printf("Reading: %d, %d\n", t1, t2);
            t1 = scratchPad[0];
            t2 = scratchPad[1];
            f=(float)((t1 + (t2*256))/16);

            if(ds18b20_reset_and_check()) {
                return(f);
            }
        }
    }

    return 0;

    // unsigned char check;
    //char temp1=0, temp2=0;

    //check=ds18b20_reset_and_check();
    //if(check)
    //{
    //    ds18b20_send_byte(0xCC);
    //    ds18b20_send_byte(0x44);
    //    nrf_delay_ms(600);
    //    ds18b20_send_byte(0xCC);
    //    ds18b20_send_byte(0xBE);
    //    temp1=ds18b20_read_byte();
    //    temp2=ds18b20_read_byte();
    //    float temp=0;
    //    temp=(float)(temp1+(temp2*256))/16;
    //    return temp;
    //}
    //  return 0;
}



/**@brief Function for reading bit.
 */
uint8_t OneWire_read_bit(void)
{
    uint8_t r;
    nrf_delay_us(2);
    nrf_gpio_cfg_output(DS18B20PIN);
    nrf_gpio_pin_clear(DS18B20PIN);
    nrf_delay_us(6);
    nrf_gpio_cfg_input(DS18B20PIN,NRF_GPIO_PIN_NOPULL);
    nrf_delay_us(3);
    r =nrf_gpio_pin_read(DS18B20PIN);
    nrf_delay_us(60);
    return r;
}


/**@brief Function for reading.
 */
uint8_t OneWire_read()
{
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( OneWire_read_bit()) r |= bitMask;
    }
    return r;
}


/**@brief Function for reading scratchpad value
 */
void ds18b20_readScratchPad(uint8_t *scratchPad, uint8_t fields)
{
    ds18b20_reset_and_check();
    ds18b20_send_byte(0xCC);
    ds18b20_send_byte(0xBE);

    for(uint8_t i=0; i < fields; i++)
    {
        scratchPad[i] = OneWire_read();
    }
    ds18b20_reset_and_check();
}


/**@brief Function for request temperature reading
 */
void ds18b20_requestTemperatures(void)
{
    ds18b20_reset_and_check();
    ds18b20_send_byte(0xCC);
    ds18b20_send_byte(0x44);
}


/**@brief Function for reading temperature method 2
 */
float ds18b20_read_temp_2(void)
{
    ds18b20_requestTemperatures();
    unsigned char check;
    nrf_delay_us(600);
    ScratchPad scratchPad;
    ds18b20_readScratchPad(scratchPad, 2);
    int rawTemperature = (((int)scratchPad[1]) << 8) + scratchPad[0];
    uint8_t sign = scratchPad[1] & 0x80;
    if(sign) {
        rawTemperature = ((rawTemperature ^ 0xffff)+1)*-1;
    }
    printf("Received %d, %d\n", scratchPad[1], scratchPad[0]);
    float temp = 0.0625 * rawTemperature;

    return temp;
}

