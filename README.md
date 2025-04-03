# ROV_Firmware

## Description

This is the firmware for 2025's ROV, Giulietta. It is designed to be used with the ESP32 microcontroller.

## User Guide

### Requirements

- [Adafruit Unified BNO055 Driver ](https://github.com/adafruit/Adafruit_BNO055)
- [Adafruit Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor)

### Pinout

| Pin | Function                        |
| --- | ------------------------------- |
| 17  | Horizontal Thruster A Direction |
| 13  | Horizontal Thruster A PWM       |
| 15  | Horizontal Thruster B Direction |
| 23  | Horizontal Thruster B PWM       |
| 16  | Horizontal Thruster C Direction |
| 4   | Horizontal Thruster C PWM       |
| 5   | Horizontal Thruster D Direction |
| 2   | Horizontal Thruster D PWM       |
| 33  | Vertical Thruster E PWM A       |
| 25  | Vertical Thruster E PWM B       |
| 27  | Vertical Thruster F PWM A       |
| 14  | Vertical Thruster F PWM B       |
| 32  | Vertical Thruster E Enable      |
| 26  | Vertical Thruster F Enable      |
| 19  | LED                             |
| 12  | DCV 1                           |
| 18  | DCV 2                           |
