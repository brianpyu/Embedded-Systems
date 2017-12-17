# Embedded-Systems
Final Project for Embedded Systems (Uncrashable Car)
This project contains code for an uncrashable RC car. The car is controlled using Bluetooth via the HC-08 
Bluetooth UART Communication Module, allowing the user to send forward, right, and left signals from an 
iPhone. The car uses a distance sensor (HC-SR04 Ultrasonic Sensor) mounted at the front to determine the 
distance between it and any objects in its path. The TIVA C Microcontroller processes signals from the iPhone
and the distance sensor, disabling forward movement when the distance sensor senses an obstacle too close to
the car. To move, the TIVA C Microcontroller lights an LED which provides an input signal to an H Bridge, which
in turn provides an output voltage across the terminals of the RC Car's DC motor. 
