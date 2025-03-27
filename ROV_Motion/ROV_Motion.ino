#include "define.h"

// Mate 2025 ROV Competition
// Team: AU-ROBOTICS

// ROV Firmware Code

// AU-ROBOTICS ROV Motion Control Code
// This code is used to control the motion of the ROV using the input forces from the PI
// The code reads the input forces from the PI and computes the thruster forces required to achieve the desired motion
// The code then applies constraints to the thruster forces and sends the PWM signals to the thrusters to achieve the desired motion

void setup() {
  Serial.begin(115200);

  // setup motors
  setup_H_motors();
  setup_V_motors();

  pinMode(led, OUTPUT);

  // Initialize valve pins
  setupDCV();

  // Initialize BNO055 sensor
  setupBno();

  // turn off all motors
  stopMotors();
  delay(2000);

#ifdef TEST_MOTORS
  testMotors();
  delay(TIME_FOR_TESTING_MOTORS);
  stopMotors();
#endif

#ifdef CHECK_ALL_SYSTEM
  checkAllSystem();
#endif
}

void loop() {
  mainC();
}
