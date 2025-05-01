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
#include "Thruster.h"

// ########################################################### Defines ########################################################### //

// Define the pins for the horizontal and vertical thrusters
// 8 thrusters (4 horizontal, 4 vertical)
// Horizontal Thrusters: A, B, C, D
// Vertical Thrusters: E, F , G, H

// === Cytron 1 H (Horizontal Thrusters Group 1) ===
#define M1_DIR      5     // Motor 1 Direction
#define M1_PWM      6     // Motor 1 Speed (PWM)
#define M2_DIR      7     // Motor 2 Direction
#define M2_PWM      8     // Motor 2 Speed (PWM)

// === General Outputs ===
#define dcv2        9     // Device Control Voltage 2
#define led         10    // ROV Lighting Control
#define dcv1        11    // Device Control Voltage 1

// === I2C Interface ===
#define I2C_SDA     12    // I2C SDA
#define I2C_SCL     15    // I2C SCL

// === Cytron 3 V (Vertical Thrusters Group 1) ===
#define M5_DIR      16    // Motor 5 Direction
#define M5_PWM      17    // Motor 5 Speed (PWM)
#define M6_DIR      18    // Motor 6 Direction
#define M6_PWM      19    // Motor 6 Speed (PWM)

// === Cytron 2 H (Horizontal Thrusters Group 2) ===
#define M3_DIR      40    // Motor 3 Direction
#define M3_PWM      41    // Motor 3 Speed (PWM)
#define M4_DIR      38    // Motor 4 Direction
#define M4_PWM      39    // Motor 4 Speed (PWM)

// === BTS Pitch Back (Vertical Thrusters Group 2) ===
#define M8_LEFT_PWM   35  // Motor 8 Left PWM
#define M8_RIGHT_PWM  36  // Motor 8 Right PWM

// === BTS Pitch Front (Vertical Thrusters Group 3) ===
#define M7_LEFT_PWM   26  // Motor 7 Left PWM
#define M7_RIGHT_PWM  27  // Motor 7 Right PWM

// === USB Pins (optional use) ===
#define USB_DPLUS     25
#define USB_DMINUS    24


// A 
#define Thruster_A_DIR          M1_DIR
#define Thruster_A_PWM          M1_PWM

// B
#define Thruster_B_DIR          M2_DIR
#define Thruster_B_PWM          M2_PWM

// C
#define Thruster_C_DIR          M3_DIR
#define Thruster_C_PWM          M3_PWM

// D    
#define Thruster_D_DIR          M4_DIR
#define Thruster_D_PWM          M4_PWM

// E
#define Thruster_E_DIR          M5_DIR
#define Thruster_E_PWM          M5_PWM

// F
#define Thruster_F_DIR          M6_DIR
#define Thruster_F_PWM          M6_PWM

// G
#define Thruster_G_left_PWM     M7_LEFT_PWM
#define Thruster_G_right_PWM    M7_RIGHT_PWM

// H
#define Thruster_H_left_PWM     M8_LEFT_PWM
#define Thruster_H_right_PWM    M8_RIGHT_PWM



// Define the indices for the input commands from the station
// Fx, Fy, Tau, Fz, Tpitch , Troll , Tyaw
#define fx_index_incoming_data      0
#define fy_index_incoming_data      1
#define fz_index_incoming_data      2
#define tpitch_index_incoming_data  3
#define troll_index_incoming_data   4
#define tyaw_index_incoming_data    5

// Define the indices for the input commands to the kinematic model
// Fx, Fy, Tau, Fz, Tpitch , Troll , Tyaw
#define fx_index_input      0
#define fy_index_input      1
#define fz_index_input      2
#define tpitch_index_input  3
#define troll_index_input   4
#define tyaw_index_input    5

// Define the maximum input values for KInematic model
// Fx, Fy, Fz , Tpitch , Troll , Tyaw
#define max_mapped_input 1020.0

// Define the maximum input range from the station 
// Fx, Fy, Fz , Tpitch , Troll , Tyaw
#define max_input_cmd 254 

// Define the sign byte index in the incoming data
#define sign_byte_index_incoming_data 5

// Define the indices for the byte containing the incoming data
#define led_index_incoming_data  6
#define dcv1_index_incoming_data 6
#define dcv2_index_incoming_data 6

// Define the indices for the bits in the byte containing the incoming data
#define led_index_input  0
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
