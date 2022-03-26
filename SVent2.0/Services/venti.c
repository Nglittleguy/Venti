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


/**@brief Function for Writing the Day and Voltage into the FDS Buffer
 *
 * @details Reads the voltage, and writes the day + voltage into flash storage
 *
 * @param[in] flash_write_buf   Buffer to fill with data to write to fds
 * @param[in] epoch             Current time in epoch seconds (keep track of day)
 */
void setDayVoltageBuffer(char* flash_write_buf, uint32_t epoch) {
    memset(flash_write_buf, 0, 8);
    uint16_t vbatt;

    //Enable voltage reading via mosfet switch, read, turn off
    voltage_read_enable(true);
    battery_voltage_get(&vbatt);
    voltage_read_enable(false);

    //If battery is less than 3500 mV, then open motor, disallow closing
    //    Can change this value depending on when "LOW BATTERY" mode begins
    if(vbatt < 3500) {
        low_battery = true;
        rotate(255);
    }

    //Write onto buffer as a uint32_t array (8 bytes total)
    uint32_t day_write_buffer [2] = {epoch, vbatt};
    memcpy(flash_write_buf, day_write_buffer, 8);
}


/**@brief Function for Writing Temperature and Time Segment into FDS
 *
 * @details Write temp_min, temp_max, time_segment into flash storage
 *
 * @param[in] flash_write_temp  Buffer to fill with data to write to fds
 * @param[in] segment           Time segment (keep track of time)
 */
void setTemperatureBuffer(char* flash_write_temp, uint16_t segment) {
    //Write onto buffer as short array (8 bytes total because word aligned)
    short temp_buffer [3] = {segment, temp_min, temp_max};
    memset(flash_write_temp, 0, 8);
    memcpy(flash_write_temp, temp_buffer, 6);
}


/**@brief Function for Writing Schedule into FDS
 *
 * @details Write schedule into flash storage
 *
 * @param[in] flash_write_schedule  Buffer to fill with data to write to fds
 */
void setScheduleBuffer(char* flash_write_schedule) {
    memset(flash_write_schedule, 0, 128);
    memcpy(flash_write_schedule, schedule, sizeof(Schedule_event)*35);
}


/**@brief Function for Switching Voltage MOSFET on/off
 *
 * @details Switch Mosfet on/off based on var
 *
 * @param[in] connect  If true, set pin, else, clear pin
 */
void voltage_read_enable(bool connect) {
    if(connect) {
        nrf_gpio_pin_set(VOLTAGE_MOSFET_PIN);
    }
    else {
        nrf_gpio_pin_clear(VOLTAGE_MOSFET_PIN);
    }
    nrf_delay_ms(10);
}


//////////////////////////////////////////////////////////////////////////////////
//Motor Services

bool lights = false;
static uint8_t current_open_amount = 255;

/**@brief Function to initialize motor pins on board
 *
 * @details Set all motor pins to output configuration (and voltage mosfet)
 */
void board_motor_init(void) {
    for(int i = 0; i<4; i++) {
        nrf_gpio_cfg_output(motor_pins[i]);
    }
    nrf_gpio_cfg_output(VOLTAGE_MOSFET_PIN);
}


/**@brief Function to reset motor to start position
 *
 * @details Set all motor pins to proper output to begin movement
 */
void reset_motor() {
    //Set the motor to the same start state
    for(int pinI = 0; pinI < 4; pinI++) {
        if(motor_reset[pinI])
           nrf_gpio_pin_set(motor_pins[pinI]);
        else 
           nrf_gpio_pin_clear(motor_pins[pinI]);
    }
}


/**@brief Function to zero out motor output
 *
 * @details Clear all motor pins (reduces current consumption)
 */
void zeroMotor() {
    for(int pinI = 0; pinI<4; pinI++) {
        nrf_gpio_pin_clear(motor_pins[pinI]);
    }
}


/**@brief Function to rotate the motor
 *
 * @details Rotate Motor half a cycle clockwise (Deprecated)
 */
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

/**@brief Function to rotate the motor
 *
 * @details Rotate Motor half a cycle counterclockwise (Deprecated)
 */
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


/**@brief Function to rotate the motor by variable amount
 *
 * @details Rotate Motor clockwise by a certain amount
 *
 * @param[in] amount   Amount to rotate by (255 is 90 degrees)
 */
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


/**@brief Function to rotate the motor by variable amount
 *
 * @details Rotate Motor counterclockwise by a certain amount
 *
 * @param[in] amount   Amount to rotate by (255 is 90 degrees)
 */
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


/**@brief Function to rotate the motor to a specific amount
 *
 * @details Rotate Motor until specified opening is 0 (closed) or 255 (open)
 *
 * @param[in] target_open_amount   Amount of openness to reach
 */
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


/**@brief Function to get the amount that the vent is currently opened
 *
 * @details Rotate Motor counterclockwise by a certain amount
 *
 * @return[out] uint8_t of current open amount
 */
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

    nrf_gpio_cfg_input(DS18B20PIN, NRF_GPIO_PIN_NOPULL);

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


/**@brief Function for reseting and checking temperature sensor
 */
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

/**@brief Function for reading the temperature as a float
 * 
 * @detail Reads temperature from sensor - do not run without temperature sensor connected
 */
float ds18b20_read_temp(void) {
    
    //Prevents breaking if no temperature sensor is attached
    if(TEMP_SENSOR_INVALID) {
        return 88;
    }

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


/**@brief Function for initializing schedule
 * 
 * @detail Schedule Event is invalid if time segment is 9999, set all invalid
 */
void initSchedule() {
    for(int i = 0; i<7; i++) {
        for(int j = 0; j<5; j++) {
            schedule[i][j].time = 9999;
        }
    }
}


/**@brief Function for printing the schedule
 * 
 * @detail Print Schedule via Log
 */
void printSchedule() {
    char c[32];
    for(int i = 0; i<7; i++) {
        NRF_LOG_INFO("%s Schedule:\n", daysOfWeek[i]);
        for(int j = 0; j<5; j++) {
            if(schedule[i][j].time==9999) {
                continue;
            }
            memset(c, 0, 32);
            currentTimeFromSegment(c, schedule[i][j].time);
            NRF_LOG_INFO("%d. %s - %d\n\r", j+1, c, schedule[i][j].amount);
        }
    }
}


/**@brief Function for putting the schedule into a buffer to send to user
 * 
 * @detail sprintf schedule into the buffer for a specified weekday
 *
 * @param[in] weekday   Day of week to fetch schedule for
 * @param[in] buf       Buffer to write to
 */
void sendSchedule(uint8_t weekday, char* buf) {
    sprintf(buf, "00@%d@%d|%d,%d|%d,%d|%d,%d|%d,%d|%d#", weekday+711, 
      schedule[weekday][0].time, schedule[weekday][0].amount, 
      schedule[weekday][1].time, schedule[weekday][1].amount,
      schedule[weekday][2].time, schedule[weekday][2].amount,
      schedule[weekday][3].time, schedule[weekday][3].amount,
      schedule[weekday][4].time, schedule[weekday][4].amount);
}


/**@brief Function for setting the schedule
 * 
 * @detail Set the schedule for this day of the week
 *
 * @param[in] weekday   Day of week to set schedule for
 * @param[in] timeArr   Array of times for events
 * @param[in] amountArr Array of open amounts for events
 */
void setSchedule(uint8_t weekday, uint16_t* timeArr, uint8_t* amountArr) {
    for(int i = 0; i<5; i++) {
        schedule[weekday][i].time = timeArr[i];
        schedule[weekday][i].amount = amountArr[i];
    }
}


/**@brief Function setting a buffer the readable time given a time segment
 * 
 * @detail Convert a time segment into the time of the week
 *
 * @param[in] buf           Buffer to write time to
 * @param[in] time_segment  weekly time segment to be converted
 */
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


/**@brief Function for adding a single event to the schedule
 * 
 * @detail Set the schedule event for the specified arguments (Deprecated)
 *
 * @param[in] slot          Which of the 5 slots of the schedule per day to set
 * @param[in] time          The time segment for the event
 * @param[in] amount        The amount the vent should open at the event
 */
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


/**@brief Function for reseting the min and max temperatures for a new hour
 * 
 * @detail Set min and max temperatures to the current temperature
 */
void resetTemperatureMinMax() {
    short temp_x_100 = (short) 100*ds18b20_read_temp();
    temp_min = (short) temp_x_100;
    temp_max = (short) temp_x_100;
}


/**@brief Function for comparing the min and max temperatures within this hour
 * 
 * @detail Set min and max temperatures based on current temperature
 */
void compareTemperatureMinMax() {
    short temp_x_100 = (short) 100*ds18b20_read_temp();
    if(temp_x_100 < temp_min) {
        temp_min = temp_x_100;
    }
    if(temp_x_100 > temp_max) {
        temp_max = temp_x_100;
    }
}


/**@brief Function for checking the schedule for vent openings
 * 
 * @detail Check through all schedules to see if vent needs to be opened
 *
 * @param[in] time_segment      Current time to check within schedule
 */
bool checkSchedule(uint16_t time_segment) {
    NRF_LOG_INFO("Time is %d", time_segment);
    uint8_t dayOfWeek = time_segment/288;
    bool hasRotated = false;

    for(int i = 0; i<5; i++) {
        //Rotate if the time matches, and not low battery
        if(schedule[dayOfWeek][i].time == time_segment && !low_battery) {
            rotate(schedule[dayOfWeek][i].amount);
            hasRotated = true;
            NRF_LOG_INFO("Rotated to %d at %d following Schedule",
                schedule[dayOfWeek][i].amount, schedule[dayOfWeek][i].time);
        }
    }
    return hasRotated;
}