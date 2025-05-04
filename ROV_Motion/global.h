#ifndef GLOBAL_H
#define GLOBAL_H

#include "define.h"

// ########################################################### Global variables ########################################################### //
// input forces (Fx, Fy, Fz , Tpitch , Troll , Tyaw) 
extern double inputCmds[6];  // input forces (Fx, Fy, Fz , Tpitch , Troll , Tyaw) 


// Output thrust values
extern float outputThrusters[8] ;

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
extern bool NULL_INPUT_ROLL_FLAG ;
extern bool dataValid;

// Time tracking
extern unsigned long last_time_data_received;
extern unsigned long last_time_data_sent;

// BNO055 IMU object
extern Adafruit_BNO055 Bno;
extern Thruster thrusters[8] ;

// Pseudoinverse matrices
extern double T_inverse_matrix[8][6];


// Function prototype 
void correctYawAngle();

// ################################################# End of Global variables ############################################################### //

#endif  // GLOBAL_H
