# PID Controllinator 2000
_A demonstration unit for Digital Control Systems_

This product is built on an STM32 Nucleo board and allows for an analogue set point and input, and PWM output. These values can be calibrated on-demand to match that of the system. The deviation of the input to the set point is displayed on an OLED display.
The PID controller can then be tuned by changing the Kp, Ki, and Kd values on the LCD screen to stabilize the system.

# Features
* Fully configurable Kp, Ki, and Kd values.
* LCD screen that shows the current set point and input error.
* Rotary encoder allows for configuration on the LCD screen.
* OLED screen showing real-time graph of input and set point.
* Control mode toggle to bypass the PID controller if desired.
* Graph of set point, input, and output available with a serial plotter.
* Battery or USB-powered.
* Voltage protection on input pins.
* Output is 10 kHz PWM signal.
* DC-to-crocodile-clip cables provide safe handling.
* EEPROM chip for saving the configuration.
