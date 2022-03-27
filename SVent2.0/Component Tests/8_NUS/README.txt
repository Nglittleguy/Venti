SVent 1.8 - Nordic UART Services (NUS) Test
- To test the NUS, download the 'nRF Connect' mobile application
- Place an LED at GPIO 29 (BL652 Pin 34).
- The negative side of the LED should be connected to a resistor of around 1k ohms (not super specific)
- The negative side of the resistor should be connected to GND


Expected Result:
- Scanning should show 'SmartLabs Test'
- Can connect
- Expand 'UART Services'
- Click on down arrow for 'TX Characteristics' to enable Notifications
- Click on up arrow for 'RX Characteristics' and send the string "on"
	- LED should turn on
	- should receive 'on received' in 'TX Characteristics'
- Click on up arrow for 'RX Characteristics' and send the string "off"
	- LED should turn off
	- should receive 'off received' in 'TX Characteristics'

