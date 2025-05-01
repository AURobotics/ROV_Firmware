#ifndef FUNC_H
#define FUNC_H

#include "define.h"

// Function prototypes

// Lights control
void turnLight(bool state);

// Setup DCVs
void setupDCV();

// setup thrysters
void setupThrusters();

// Thrusters control
void controlThrusters();

// Thruster force computation
void applyConstraints(float *thruster_forces, int size, float max_force);

// Compute thrust speeds
void calculateThrust(double *input, double T_inverse[8][6], float *outputThrusters)

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
