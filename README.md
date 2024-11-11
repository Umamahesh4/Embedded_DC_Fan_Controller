This project demonstrates a temperature-controlled DC fan system using an STM32F103C8 microcontroller. The system monitors ambient temperature and adjusts fan speed accordingly. An LCD display shows ADC data, and an LED indicates when the fan is active or not .


Overview
The Embedded DC Fan Controller project leverages the STM32F103C8 microcontroller to control a DC fan based on temperature readings. An onboard ADC reads temperature data, and depending on these values, the fan and LED are activated. The system includes an LCD display connected through I2C for real-time monitoring of temperature readings.

There are two switches:
1. one for controlling the fan irrespective of temperature data ( using the interrupt) 
2. to shutdown the entire power supply( emergency shutdown)

Features
Temperature Monitoring: Inbuilt temperature sensor with ADC to detect ambient temperature.
Fan Control: Automatically activates the fan and LED when temperature exceeds a set threshold.
LCD Display (16x2): Displays real-time temperature data using an I2C module.
User Adjustments: A potentiometer allows adjustment of temperature thresholds.
Hardware Components
STM32F103C8 Microcontroller
16x2 LCD Display (with I2C module)
UL2003 Register (for fan control)
DC Fan
LED (for fan status indication)
Potentiometer (to adjust threshold temperature)



![Circuit Diagram1](embedded3)
------------------------------------------------------------------------------------
![Circuit Diagram1](embedded2)
