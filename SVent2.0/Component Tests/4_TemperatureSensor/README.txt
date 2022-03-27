SVent 1.4 - Temperature Sensor Test
- To test the Temperature Sensor, connect the DS18B20 temperature sensor to VDD and GND
- Attach the 1-wire digital pin to GPIO 9 (BL652 Pin 15)
- Attach a 4.7k ohm resistor connecting the 1-wire digital pin and VDD (ensure that this is correct)


Expected Result:
- Every 2 seconds, the NRF RTT debug screen should print out the current temperature in Celsius

Potential Fixes:
- If result is 88.0, then check 'Services/venti.h' to ensure that the 'TEMP_SENSOR_INVALID' is set to '0'
- If result is 85.0, then that means that the sensor did not have enough time to save the temperature since reset
