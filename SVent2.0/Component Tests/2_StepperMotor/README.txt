SVent 1.2 - Stepper Motor Test
- To test the Stepper Motor, output from GPIO 15, 17, 19, 31 (BL652 Pins 29, 30, 31, 32)
- In that order, connect to Pins 1, 2, 3, 4 of the ULN2003AN Integrated Circuit Motor Driver
- Output ULN2003AN Pins 16, 15, 14, 13 to 28BYJ-48 Stepper Motor Inputs 1, 2, 3, 4


Expected Result:
- Motor will rotate half a full rotation, and then rotate half a rotation in the other direction
	This will repeat forever.


Potential Fix:
- If the motor is turning in 1 direction, and then refuses to turn
			or
- If the motor is continually turning in 1 direction

- Then in 'Services/venti.c', Line 181 and Line 225 should have the for loop go in the opposite order
	- ex. 'for(int pinI = 0; pinI < 4; pinI++)' -> 'for(int pinI = 3; pinI>=0; pinI--)' 
