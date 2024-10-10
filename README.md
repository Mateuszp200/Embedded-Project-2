# Embedded-Project-2
Project Target:
Write a single ATmega4809 program that to run on an Arduino Nano Every with a Micro Servo Motor and a UL OER Shield that will:
•	Move a servomotor at a speed determined by the user, where the servo motor position is set using PWM.
•	Measure the period, and high and low pulse widths of an input signal applied to Port E bit 3 (used as the Input Capture pin) and report the reading to the user. If the external chip producing this signal has stopped oscillating for a time, report this to the user when pulse widths or signal periods are requested.
•	Turn on or off Bits 2, 3 & 4 of the LED array based on the time measured on PortE bit 3 (the Input Capture pin)
•	Read the analog voltage on the AIN3 input and report the reading to the user. Set or clear Bit 6 of the LED array based on the ADC reading.
