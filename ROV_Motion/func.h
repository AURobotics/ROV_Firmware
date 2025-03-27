#ifndef FUNC_H
#define FUNC_H

#include "define.h"

// Function prototypes

// Lights control
void turnLight(bool state);

// Thrusters setup
void setup_H_motors();
void setup_V_motors();
void setupDCV();

// Thrusters control
void controlHmotors();
void controlVmotors();
void controlMotors();

// Thruster force computation
void applyConstraints(float *thruster_forces, int size, float max_force);
void ComputeHorrizontalThrustForces(double *input, double T_inverse[4][3], float *outputThrusters);
void ComputeVerticalThrustForces(double *input, double T_inverse[2][2], float *outputThrusters);
void calculateThrust();

// Serial communication and control
void readIncomingData();
void operatePID();

// DC Valve control
void dcv1Control(bool state);
void dcv2Control(bool state);

// IMU functions
void imu_read();
void setupBno();

// Debugging functions
void debugThrusters();
void debugSensors();
void checkPitchPid();
void checkYawPid();

// Motor control
void stopMotors();
void testMotors();
void checkSerial();
void checkAllSystem();
void checkSerialDataAndControlMotors();

// Data transmission
void sendSensorData();

// Main function
void mainC();

#endif // FUNC_H
