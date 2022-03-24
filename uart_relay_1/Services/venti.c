#include "venti.h"

//////////////////////////////////////////////////////////////////////////////////
//Battery & Blinking LED Services

bool low_battery = false;

//////////////////////////////////////////////////////////////////////////////////
//Flash Storage Services

const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

/**@brief   Begin deleting all records, one by one. */
void delete_all_begin(void)
{
    m_delete_all.delete_next = true;
}

bool record_delete_next(void)
{
    fds_find_token_t  tok   = {0};
    fds_record_desc_t desc  = {0};

    if (fds_record_iterate(&desc, &tok) == NRF_SUCCESS)
    {
        ret_code_t rc = fds_record_delete(&desc);
        if (rc != NRF_SUCCESS)
        {
            return false;
        }

        return true;
    }
    else
    {
        /* No records left to delete. */
        return false;
    }
}

/**@brief   Process a delete all command.
 *
 * Delete records, one by one, until no records are left.
 */
void delete_all_process(void)
{
    if (   m_delete_all.delete_next
        & !m_delete_all.pending)
    {
        NRF_LOG_INFO("Deleting next record.");

        m_delete_all.delete_next = record_delete_next();
        if (!m_delete_all.delete_next)
        {
            NRF_LOG_INFO("No records left to delete.");
        }
    }
}



void setDayVoltageBuffer(char* flash_write_buf, uint32_t epoch) {
    memset(flash_write_buf, 0, 36);
    uint16_t vbatt;
    battery_voltage_get(&vbatt);

    //If battery is less than 3.5V, then open motor, disallow closing
    if(vbatt < 3500) {
        low_battery = true;
        rotate(255);
    }
    sprintf(flash_write_buf, "Time: %d - %d mV", epoch, vbatt);
}

void setTemperatureBuffer(char* flash_write_temp, uint16_t segment) {
    memset(flash_write_temp, 0, 36);
    sprintf(flash_write_temp, "Segment %d: %d - %d", segment, temp_min, temp_max);
}

void setScheduleBuffer(char* flash_write_schedule, uint32_t current_epoch_sec) {
    memset(flash_write_schedule, 0, 128);
    memcpy(flash_write_schedule, schedule, sizeof(Schedule_event)*35);
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
        nrf_gpio_cfg_output(motor_pins[i]);
    }
}

void reset_motor() {
    //Set the motor to the same start state
    for(int pinI = 0; pinI < 4; pinI++) {
        if(motor_reset[pinI])
           nrf_gpio_pin_set(motor_pins[pinI]);
        else 
           nrf_gpio_pin_clear(motor_pins[pinI]);
    }
}

void zeroMotor() {
    for(int pinI = 0; pinI<4; pinI++) {
        nrf_gpio_pin_clear(motor_pins[pinI]);
    }
}

void rotateCWHalf() {
    reset_motor();
    for(int step = 0; step<STEPCOUNT; step++) {
        for(int pinI = 0; pinI < 4; pinI++) {
            if(cw_seq[step%4][pinI])
                nrf_gpio_pin_set(motor_pins[pinI]);
            else 
                nrf_gpio_pin_clear(motor_pins[pinI]);
        }
        nrf_delay_ms(STEPDELAY);
    }
    zeroMotor();
}

void rotateCCWHalf() {
    reset_motor();
    for(int step = 0; step<STEPCOUNT; step++) {

    //The CCW pattern must be set backwards... for some reason
        for(int pinI = 0; pinI <4; pinI++) {
            if(ccw_seq[step%4][pinI])
                nrf_gpio_pin_set(motor_pins[pinI]);
            else 
                nrf_gpio_pin_clear(motor_pins[pinI]);
        }
        nrf_delay_ms(STEPDELAY);
    }
    zeroMotor();
}


static uint8_t current_open_amount = 255;


void rotateCW(short amount) {
    reset_motor();
    for(int step = 0; step<(amount<<1); step++) {
        for(int pinI = 0; pinI < 4; pinI++) {
            if(cw_seq[step%4][pinI])
                nrf_gpio_pin_set(motor_pins[pinI]);
            else 
                nrf_gpio_pin_clear(motor_pins[pinI]);
        }
        nrf_delay_ms(STEPDELAY);
    }
    zeroMotor();
}

void rotateCCW(short amount) {
    reset_motor();
    for(int step = 0; step<(amount<<1); step++) {

    //The CCW pattern must be set backwards... for some reason
        for(int pinI = 0; pinI < 4; pinI++) {
            if(ccw_seq[step%4][pinI])
                nrf_gpio_pin_set(motor_pins[pinI]);
            else 
                nrf_gpio_pin_clear(motor_pins[pinI]);
        }
        nrf_delay_ms(STEPDELAY);
    }
    zeroMotor();
}

void rotate(uint8_t target_open_amount) {
    
    short steps_to_take = current_open_amount - target_open_amount;
    current_open_amount = target_open_amount;
    if(steps_to_take < 0) {
        rotateCCW(-steps_to_take);
    }
    else {
        rotateCW(steps_to_take);
    }
}

uint8_t getOpenAmount() {
    return current_open_amount;;
}

/////////////////////////////////////////////////////////////////////////////////
//Temperature Sensor - https://github.com/DSysoletin/nrf52_ds18b20_example/blob/master/main.c

short temp_max = SHRT_MIN;
short temp_min = SHRT_MAX;


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

    if(nrf_gpio_pin_read(DS18B20PIN)) {
        presence = 1;
    }
    else {
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
    return 8888;   //TESTING, PLEASE REMOVE


    char t1 = 0, t2 = 0;
    double f;
    uint8_t scratchPad[9];
    if(ds18b20_reset_and_check()) {

        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0x44);
        nrf_delay_ms(410);

        if(ds18b20_reset_and_check()) {

            ds18b20_send_byte(0xCC);
            ds18b20_send_byte(0xBE);

            for(uint8_t i = 0; i<9; i++) {
                scratchPad[i] = ds18b20_read_byte();
                //printf("Scratch %d: %x\n", i, scratchPad[i]);
            }

            t1 = scratchPad[0];
            t2 = scratchPad[1];

            f = ((float)(t1 + (t2*256))/16);

            if(ds18b20_reset_and_check()) {
                return(f);
            }
        }
    }

    return 0;
}


/////////////////////////////////////////////////////////////////////////////////
//Scheduling System

Schedule_event schedule[7][5];

void initSchedule() {
    for(int i = 0; i<7; i++) {
        for(int j = 0; j<5; j++) {
            schedule[i][j].time = 9999;
        }
    }
}

void printSchedule() {
    char c[32];
    for(int i = 0; i<7; i++) {
        printf("%s Schedule:\n", daysOfWeek[i]);
        for(int j = 0; j<5; j++) {
            if(schedule[i][j].time==9999) {
                continue;
            }
            memset(c, 0, 32);
            currentTimeFromSegment(c, schedule[i][j].time);
            printf("%d. %s - %d\n\r", j+1, c, schedule[i][j].amount);
        }
    }
}

void sendSchedule(uint8_t weekday, char* buf) {
    sprintf(buf, "00@%d@%d|%d,%d|%d,%d|%d,%d|%d,%d|%d#", weekday+711, 
      schedule[weekday][0].time, schedule[weekday][0].amount, 
      schedule[weekday][1].time, schedule[weekday][1].amount,
      schedule[weekday][2].time, schedule[weekday][2].amount,
      schedule[weekday][3].time, schedule[weekday][3].amount,
      schedule[weekday][4].time, schedule[weekday][4].amount);
}

void setSchedule(uint8_t weekday, uint16_t* timeArr, uint8_t* amountArr) {
    for(int i = 0; i<5; i++) {
        schedule[weekday][i].time = timeArr[i];
        schedule[weekday][i].amount = amountArr[i];
        
    }
    
}

void currentTimeFromSegment(char* buf, uint16_t time_segment) {
    if(time_segment>2015) {
        sprintf(buf, "Error in Time Segment %d", time_segment);
        return;
    }
    uint8_t dayOfWeek = time_segment/288;
    uint16_t dayTime = time_segment%288;
    uint16_t hour = dayTime/12;
    uint16_t minute = 5*(dayTime%12);
    sprintf(buf, "Time: %s %02d:%02d", daysOfWeek[dayOfWeek], hour, minute); 
}


void addToSchedule(uint8_t slot, uint16_t time, uint8_t amount) {
    uint8_t dayOfWeek = time/288;
    if(slot>5 || slot == 0 || time>2015) {
        printf("Error - Slot or Time exceeds bounds");
    }
    schedule[dayOfWeek][slot-1].time = time;
    schedule[dayOfWeek][slot-1].amount = amount;
}




/////////////////////////////////////////////////////////////////////////////////
//Timer for Events

void resetTemperatureMinMax() {
    short temp_x_100 = (short) 100*ds18b20_read_temp();
    temp_min = (short) temp_x_100;
    temp_max = (short) temp_x_100;
}

void compareTemperatureMinMax() {
    short temp_x_100 = (short) 100*ds18b20_read_temp();
    if(temp_x_100 < temp_min) {
        temp_min = temp_x_100;
    }
    if(temp_x_100 > temp_max) {
        temp_max = temp_x_100;
    }
}

bool checkSchedule(uint16_t time_segment) {
    NRF_LOG_INFO("Time is %d", time_segment);
    uint8_t dayOfWeek = time_segment/288;
    bool hasRotated = false;

    for(int i = 0; i<5; i++) {
        if(schedule[dayOfWeek][i].time == time_segment && !low_battery) {
            rotate(schedule[dayOfWeek][i].amount);
            hasRotated = true;
            NRF_LOG_INFO("Rotated to %d at %d following Schedule",
                schedule[dayOfWeek][i].amount, schedule[dayOfWeek][i].time);
        }
    }
    return hasRotated;
}