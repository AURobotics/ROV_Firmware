#include "global.h"

// Input forces (Fx, Fy, Tau, Fz, Tpitch)
double inputH[3] = { 0, 0, 0 };
double inputV[2] = { 0, 0 };

// Horizontal thrust motor driver pins
int HorizontalThrusterPinsDir[4] = { dirH1, dirH2, dirH3, dirH4 };
int HorizontalThrusterPinsSpeed[4] = { pwmH1, pwmH2, pwmH3, pwmH4 };

// Vertical thrust motor driver pins
int VerticalThrusterSp1[2] = { pwmV1_1, pwmV2_1 };
int VerticalThrusterSp2[2] = { pwmV1_2, pwmV2_2 };

// Output thrust values
float outputHorizontalThrusters[4] = { 0, 0, 0, 0 };
float outputVerticalThrusters[2] = { 0, 0 };

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
bool dataValid = 0;

// Time tracking
unsigned long last_time_data_received = 0;

// BNO055 IMU object
Adafruit_BNO055 Bno(1, BNO055_ADDRESS_B);

// Pseudoinverse matrices
double T_inverse_Horizontal[4][3] = {
  { 0.25, -0.25, 1.4706 },
  { 0.25, 0.25, -1.4706 },
  { -0.25, 0.25, 1.4706 },
  { -0.25, -0.25, -1.4706 }
};

double T_inverse_Vertical[2][2] = {
  { 0.5, 2.5 },
  { 0.5, -2.5 }
};

// Function implementation
void correctYawAngle() {
  if (yawAngle > 0 && yawAngle <= 180) {
    yawAngle = -yawAngle;
  } else if (yawAngle > 180 && yawAngle <= 360) {
    yawAngle = -yawAngle - 360;
  }
}
