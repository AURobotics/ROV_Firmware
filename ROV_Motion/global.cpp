#include "global.h"

// input forces (Fx, Fy, Fz , Tpitch , Troll , Tyaw) 
double inputCmds[6]; = { 0, 0, 0, 0, 0, 0};

 
// Output thrust values
float outputThrusters[8] = { 0, 0, 0, 0 ,0 , 0, 0, 0 };


// Communication
unsigned char incoming[incoming_data_length];
unsigned char terminator = 255;

// LED and valve states
bool ledState = 1;
bool dcv1State = 0;
bool dcv2State = 0;

// IMU data
float rollAngle = 0;
float pitchAngle = 0;
float yawAngle = 0;

bool NULL_INPUT_YAW_FLAG = 0;
bool NULL_INPUT_PITCH_FLAG = 0;
bool NULL_INPUT_ROLL_FLAG = 0;
bool dataValid = 0;

// Time tracking
unsigned long last_time_data_received = 0;
unsigned long last_time_data_sent = 0;

// BNO055 IMU object
Adafruit_BNO055 Bno(1, BNO055_ADDRESS_B);

// Pseudoinverse matrices
double T_inverse_matrix[8][6] = {
  { 0.25, -0.25, 1.4706 },
  { 0.25, 0.25, -1.4706 },
  { -0.25, 0.25, 1.4706 },
  { -0.25, -0.25, -1.4706 }
};

// Function implementation
void correctYawAngle() {
  if (yawAngle > 0 && yawAngle <= 180) {
    yawAngle = -yawAngle;
  } else if (yawAngle > 180 && yawAngle <= 360) {
    yawAngle = -yawAngle + 360;
  }
}
