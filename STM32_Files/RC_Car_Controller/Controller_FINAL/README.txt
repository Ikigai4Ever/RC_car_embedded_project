# README

## HOW TO USE

This is used to control the seeker module using the STM32-L476RG Nucleo development board
A joystick, buttons, and a potentiometer are used to control the direction of movement,
the control mode (autonomous or manual), and speed of the car respectively. This project
is all based on the .ioc file pinout that is attached in the project file along
with all the settings already made.

To connect, follow the pinnout and attach the Tx from the STM32 to the Rx pin of the ESP32
in order to have the values from the controller be sent to the seeker module. This device 
is already preassembled as a unit on a custom built PCB for the project that has all 
devices hard-wired and attached already. Devices and parts can be removed and added
with ease incase of any errors or issues with different parts of the controller.