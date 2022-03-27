SVent 1.5 - Voltage Sensor Test
- To test the Voltage SAADC Sensor, connect the voltage in question to AIN 6 (BL652 Pin 33)


Expected Result:
- Every 2 seconds, the NRF RTT debug screen should print out the current voltage in mV.

Potential Fixes:
- To set the Analog Input, go change the argument in 'Services/battery_voltage_saadc.c' Line 80
	- try 'NRF_SAADC_INPUT_VDD' to check VDD