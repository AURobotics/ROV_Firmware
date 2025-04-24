#ifndef DEFINE_H
#define DEFINE_H

#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include "global.h"
#include "debug.h"
#include "pid.h"
#include "func.h"

// ########################################################### Defines ########################################################### //

// Define the pins for the horizontal and vertical thrusters
// 6 thrusters (4 horizontal, 2 vertical)
// Horizontal Thrusters: A, B, C, D
// Vertical Thrusters: E, F

#define motor1Dir 17
#define motor1Pwm 13

#define motor2Dir 15
#define motor2Pwm 23

#define motor3Dir 16
#define motor3Pwm 4

#define motor4Dir 5
#define motor4Pwm 2

#define motor5PwmA 33
#define motor5PwmB 25

#define motor6PwmA 27
#define motor6PwmB 14

// A //-> written D // motor 1
#define dirH1 motor3Dir
#define pwmH1 motor3Pwm

// B // -> written D
#define dirH2 motor4Dir
#define pwmH2 motor4Pwm

// C // written A
#define dirH3 motor2Dir
#define pwmH3 motor2Pwm

// D // written B
#define dirH4 motor1Dir
#define pwmH4 motor1Pwm

// E
#define enV1 32
#define pwmV1_1 motor5PwmA
#define pwmV1_2 motor5PwmB

// F
#define enV2 26
#define pwmV2_1 motor6PwmA
#define pwmV2_2 motor6PwmB

// Light
#define led 19

// DCV 1
#define dcv1 12

// DCV 2
#define dcv2 18

// Define the indices for the input forces
// Fx, Fy, Tau, Fz, Tpitch
#define fx_index_incoming_data 0
#define fy_index_incoming_data 1
#define tau_index_incoming_data 3
#define fz_index_incoming_data 4
#define tpitch_index_incoming_data 2

// Define the indices for the input forces
// Fx, Fy, Tau, Fz, Tpitch
#define fx_index_inputH 0
#define fy_index_inputH 1
#define tau_index_inputH 2
#define fz_index_inputV 0
#define tpitch_index_inputV 1

// Define the maximum input values for the forces
// Fx, Fy, Tau, Fz, Tpitch
#define fx_max_input 1020.0
#define fy_max_input 1020.0
#define tau_max_input 173.4
#define fz_max_input 512.0
#define tpitch_max_input 102.0

// Define the maximum output values for the forces
// Fx, Fy, Tau, Fz, Tpitch
#define fx_max_output 254.0
#define fy_max_output 254.0
#define tau_max_output 254.0
#define fz_max_output 254.0
#define tpitch_max_output 254.0

// Define the sign byte index in the incoming data
#define sign_byte_index_incoming_data 5

// Define the indices for the incoming data
#define led_index_incoming_data 6
#define dcv1_index_incoming_data 6
#define dcv2_index_incoming_data 6

// Define the indices for the incoming data
#define led_index_input 0
#define dcv1_index_input 1
#define dcv2_index_input 2

// Define the length of the incoming data from the PI (in bytes)
#define incoming_data_length 9

// Define the serial timeout in milliseconds
#define serial_timeout_ms 1000

#define serial_feedback_ms 33

#define TIME_FOR_TESTING_MOTORS 5000

// Axis remap configuration
// Default: X Axis = X, Y Axis = Y, Z Axis = Z (AXIS_REMAP_CONFIG = 0x24)
// Remap values: 00 = X, 01 = Y, 10 = Z, 11 = Invalid
#define AXIS_REMAP_X 0b01  // X-Axis
#define AXIS_REMAP_Y 0b00  // Y-Axis
#define AXIS_REMAP_Z 0b10  // Z-Axis

#define AXIS_REMAP_CONFIG ((AXIS_REMAP_Z << 4) | (AXIS_REMAP_Y << 2) | AXIS_REMAP_X)

// Axis remap sign configuration
// Default: Positive for all axes (REMAP_SIGN = 0x00)
// Remap sign values: 0 = Positive, 1 = Negative
#define AXIS_REMAP_SIGN_X 0b0  // Positive X-Axis
#define AXIS_REMAP_SIGN_Y 0b0  // Positive Y-Axis
#define AXIS_REMAP_SIGN_Z 0b1  // Positive Z-Axis

#define AXIS_REMAP_SIGN ((AXIS_REMAP_SIGN_X << 2) | (AXIS_REMAP_SIGN_Y << 1) | AXIS_REMAP_SIGN_Z)

// ############################################ End of Defines ########################################################## //

#endif // DEFINE_H
