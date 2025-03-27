#ifndef GLOBAL_H
#define GLOBAL_H

#include "define.h"

// ########################################################### Global variables ########################################################### //
// Input forces (Fx, Fy, Tau, Fz, Tpitch)
extern double inputH[3];  // input forces (Fx, Fy, Tau)
extern double inputV[2];  // input forces (Fz, Tpitch)

// Horizontal thrust motor driver pins
extern int HorizontalThrusterPinsDir[4];
extern int HorizontalThrusterPinsSpeed[4];

// Vertical thrust motor driver pins
extern int VerticalThrusterSp1[2];
extern int VerticalThrusterSp2[2];

// Output thrust values
extern float outputHorizontalThrusters[4];
extern float outputVerticalThrusters[2];

// Communication
extern unsigned char incoming[];
extern unsigned char terminator;

// LED and valve states
extern bool ledState;
extern bool dcv1State;
extern bool dcv2State;

// IMU data
extern float rollAngle;
extern float pitchAngle;
extern float yawAngle;

extern bool NULL_INPUT_YAW_FLAG;
extern bool NULL_INPUT_PITCH_FLAG;
extern bool dataValid;

// Time tracking
extern unsigned long last_time_data_received;

// BNO055 IMU object
extern Adafruit_BNO055 Bno;

// Pseudoinverse matrices
extern double T_inverse_Horizontal[4][3];
extern double T_inverse_Vertical[2][2];

// Function prototype
void correctYawAngle();

// ################################################# End of Global variables ############################################################### //

#endif  // GLOBAL_H
