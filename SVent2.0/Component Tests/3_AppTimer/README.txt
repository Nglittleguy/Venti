SVent 1.3 - App Timer Test
- To test the App Timer, place an LED at GPIO 29 (BL652 Pin 34).
- The negative side of the LED should be connected to a resistor of around 1k ohms (not super specific)
- The negative side of the resistor should be connected to GND


Expected Result:
- LED will turn on for 1 sec, then off. This will repeat forever.
- This is now done via App Timer, which lets the CPU idle - keeping track of time without busy wait
