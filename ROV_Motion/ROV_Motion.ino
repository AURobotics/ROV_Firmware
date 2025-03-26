#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>

// Mate 2025 ROV Competition
// Team: AU-ROBOTICS

// ROV Firmware Code

// AU-ROBOTICS ROV Motion Control Code
// This code is used to control the motion of the ROV using the input forces from the PI
// The code reads the input forces from the PI and computes the thruster forces required to achieve the desired motion
// The code then applies constraints to the thruster forces and sends the PWM signals to the thrusters to achieve the desired motion

//####################################################### Debugging Defines ########################################################### //

// un comment what you need to debug 
//#define DEBUG_PID_YAW 1
#define DEBUG_PID_PITCH 1
#define DEBUG_THRUSTERS 1
//#define TEST_MOTORS 1
//#define CHECK_ALL_SYSTEM 1


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
#define dirH1 motor1Dir
#define pwmH1 motor1Pwm


// B // -> written D
#define dirH2 motor2Dir
#define pwmH2 motor2Pwm

// C // written A
#define dirH3 motor3Dir
#define pwmH3 motor3Pwm

// D // written B
#define dirH4 motor4Dir
#define pwmH4 motor4Pwm

// E
#define enV1 32
#define pwmV1_1 motor5PwmA
#define pwmV1_2 motor5PwmB

// F
#define enV2 26
#define pwmV2_1 motor6PwmA
#define pwmV2_2 motor6PwmB

// Light
#define light 19

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

#define TIME_FOR_TESTING_MOTORS 5000

// ############################################ End of Defines ########################################################## //

// ########################################################### Global variables ########################################################### //
// *currently our payload scheme is (in bytes):
// **1- X-axis (fwd/bwd)                                --> 0
// **2- Y-axis (right/left)                             --> 1
// **3- Rotation about Y axis (tilt up/down)            --> 2
// **4- Rotation about Z-axis (rotating right/left)     --> 3
// **5- Z-axis (climb/descend)                          --> 4
// **6- Signs for the previous ones (5 bits)            --> 5
// **7- Leds/ valves (3 bits)                           --> 6
// **8- XOR of all the above                            --> 7
// **9- 255                                             --> 8

/*
      l-stick vert: surge (forward)
      l-stick horz: sway (right)

      r-stick vert: pitch
      r-stick horz: yaw

      L2, R2: elevation

    */

// Input forces (Fx, Fy, Tau, Fz, Tpitch)

double inputH[3] = { 0, 0, 0 };  // input forces (Fx, Fy, Tau)
double inputV[2] = { 0, 0 };     // input forces (Fz, Tpitch)

// The cytron motor driver has 2 pins for each motor: direction and speed
// The direction pins control the direction of the motor (forward or reverse) (HIGH or LOW)
// The speed pins control the speed of the motor (PWM signal)

// Horizontal thrust direction pins (A , B ,C ,D) for cyttron motor driver
int HorizontalThrusterPinsDir[4] = { dirH1, dirH2, dirH3, dirH4 };

// Horizontal thrust speeds ( A , B , C , D  ) for cyttron motor driver
int HorizontalThrusterPinsSpeed[4] = { pwmH1, pwmH2, pwmH3, pwmH4 };

// The bts7960 motor driver has 2 pins for each motor: speed and enable
// The speed pins control the speed of the motor (PWM signal)
// The enable pins enable the motor driver

// sp1 controls clockwise rotation and sp2 controls counter-clockwise rotation

// vertical thrust E speed for bts7960
int VerticalThrusterSp1[2] = { pwmV1_1, pwmV2_1 };

// vertical thrust F speed for bts7960
int VerticalThrusterSp2[2] = { pwmV1_2, pwmV2_2 };

// The computed values for the thrusters are kept in this array

// Output thrust values for horizontal
float outputHorizontalThrusters[4] = { 0, 0, 0, 0 };

// output thrust values for vertical
float outputVerticalThrusters[2] = { 0, 0 };

// The computed values for the thrusters are kept in this array


// Communication
// Define the length of the incoming data from the PI (in bytes)

// incomming Data from PI
unsigned char incoming[incoming_data_length];

// terminator for the incoming data
unsigned char terminator = 255;

// LED state
bool ledState = 1;

// DC valve 1 state
bool dcv1State = 0;

// DC valve 2 state
bool dcv2State = 0;

// IMU data
float rollAngle = 0;
float pitchAngle = 0;
float yawAngle = 0;

bool NULL_INPUT_YAW_FLAG = 0;
bool NULL_INPUT_PITCH_FLAG = 0;

bool dataValid = 0;

// Time since the last data was received
unsigned long last_time_data_received = 0;

// Create BNO055 object
Adafruit_BNO055 Bno(1, BNO055_ADDRESS_B);


void correctYawAngle() {
  if (yawAngle > 0 && yawAngle <= 180) {
    yawAngle = yawAngle;
  } else if (yawAngle > 180 && yawAngle <= 360) {
    yawAngle = yawAngle - 360;
  }
}
// Pseudoinverse matrix T_inverse for FX , FY ,YAW
double T_inverse_Horizontal[4][3] = {
  { 0.25, -0.25, 1.4706 },
  { 0.25, 0.25, -1.4706 },
  { -0.25, 0.25, 1.4706 },
  { -0.25, -0.25, -1.4706 }
};
// Pseudoinverse matrix T_inverse for FZ , PITCH
double T_inverse_Vertical[2][2] = {
  { 0.5, 2.5 },
  { 0.5, -2.5 }
};


// ########################################################### End of gloabl variables ########################################################### //


// ########################################################### PID Controllers ########################################################### //

// ############################################################## YAW PID ############################################################## //
// yaw parameters
float inputYaw = 0, kpYaw = 0.7, kiYaw = 0.0, kdYaw = 0.1, setpointYaw = 0, outputYaw = 0;
float maxOutputYaw = 173.4, minOutputYaw = -173.4;

// flag to see if the PID controller is active or not
bool flag_YAW_PID = false;

// PID controller for YAW
/**
 * PID_YAW - PID controller for YAW motion control.
 * 
 * This function calculates the output for the YAW motion control based on the 
 * setpoint and the current input. It uses a PID controller to minimize the error 
 * between the setpoint and the input. The function should be called periodically 
 * to update the output.
static unsigned long lastTime = 0;
unsigned long now = millis();
double dt = (now - lastTime) / 1000.0;
lastTime = now;
 * @param start_YAW_PID - If true, the PID controller is active and updates the output.
 *                        If false, the PID controller is reset and the output is set to zero.
 */
void PID_YAW(bool start_YAW_PID) {
  // declare variables
  double error;
  double derror;
  static float ierror = 0;
  static double prvError = 0;
  double dt;
  static unsigned long prvMillis = 0;
  if (start_YAW_PID) {
    unsigned long currentMillis = millis();
    if (currentMillis - prvMillis >= 10) {
      // Calculate the delta time
      dt = (currentMillis - prvMillis) / 1000.0;
      // Update the previous time
      prvMillis = currentMillis;
      // Calculate the error
      error = (double)(setpointYaw - inputYaw);
      // Calculate the derivative of the error
      derror = (error - prvError) / dt;
      // Update the previous error
      prvError = error;

      // Calculate the integral of the error
      if (outputYaw < maxOutputYaw && outputYaw > minOutputYaw) {
        ierror += error * dt;
      }

      // Calculate the output
      outputYaw = kpYaw * error + kiYaw * ierror + kdYaw * derror;

      // constrain the output
      outputYaw = constrain(outputYaw, minOutputYaw, maxOutputYaw);

#ifdef DEBUG_PID_YAW
      Serial.print("dt : ");
      Serial.print(dt);
      Serial.print(" Error : ");
      Serial.print(error);
      Serial.print(" dError : ");
      Serial.print(derror);
      Serial.print(" prvError : ");
      Serial.print(prvError);
      Serial.print(" setpoint : ");
      Serial.print(setpointYaw);
      Serial.print(" Yaw input : ");
      Serial.print(inputYaw);
      Serial.print(" Yaw output : ");
      Serial.println(outputYaw);
#endif
    }
  } else {
    // Reset the PID controller when start_YAW_PID is false
    ierror = 0;
    prvError = 0;
    outputYaw = 0;
  }
}

// ############################################################## PITCH PID ############################################################## //
// pitch parameters
float inputPitch = 0, kpPitch = 1.1, kiPitch = 0, kdPitch = 0, setpointPitch = 0, outputPitch = 0;
float maxOutputPitch = 102, minOutputPitch = -102;

// flag to see if the PID controller is active or not
bool flag_PITCH_PID = false;
// PID controller for PITCH
/** 
 * PID_PITCH - PID controller for PITCH motion control.
 * 
 * This function calculates the output for the PITCH motion control based on the 
 * setpoint and the current input. It uses a PID controller to minimize the error 
 * between the setpoint and the input. The function should be called periodically 
 * to update the output.
 * 
 * @param start_PITCH_PID - If true, the PID controller is active and updates the output.
 *                          If false, the PID controller is reset and the output is set to zero.
 */

void PID_PITCH(bool start_PITCH_PID) {
  // declare variables
  double error;
  double derror;
  static float ierror = 0;
  static double prvError;
  double dt;
  static unsigned long prvMillis = 0;
  unsigned long currentMillis = millis();
  if (start_PITCH_PID) {

    if (currentMillis - prvMillis >= 10) {
      // Calculate the delta time
      dt = (currentMillis - prvMillis) / 1000.0;
      // Update the previous time
      prvMillis = millis();
      // Calculate the error
      error = setpointPitch - inputPitch;
      // Calculate the derivative of the error
      derror = (error - prvError) / dt;
      // Update the previous error
      prvError = error;
      // Calculate the integral of the error
      // Check if the output is within the limits
      if (outputPitch < maxOutputPitch && outputPitch > minOutputPitch) {
        ierror += error * dt;
      }

      // Calculate the output

      outputPitch = kpPitch * error + kiPitch * ierror + kdPitch * derror;

      // constrain the output
      outputPitch = constrain(outputPitch, minOutputPitch, maxOutputPitch);
  #ifdef DEBUG_PID_PITCH
      Serial.print("dt : ");
      Serial.print(dt);
      Serial.print(" Error : ");
      Serial.print(error);
      Serial.print(" dError : ");
      Serial.print(derror);
      Serial.print(" prvError : ");
      Serial.print(prvError);
      Serial.print(" setpoint : ");
      Serial.print(setpointPitch);
      Serial.print(" Pitch input : ");
      Serial.print(inputPitch);
      Serial.print(" Pitch output : ");
      Serial.println(outputPitch);
  #endif
    }
  } else {
    // Reset the PID controller
    ierror = 0;
    prvError = 0;
    outputPitch = 0;
  }
}

// ########################################################### End of PID Controllers ########################################################### //

// ########################################################### Functions ########################################################### //

// Lights control
void turnLight(bool state) {
  analogWrite(light, state ? 127 : 0);
}

// setup horizontal thrusters

void setup_H_motors() {

  pinMode(dirH1, OUTPUT);
  pinMode(pwmH1, OUTPUT);

  pinMode(dirH2, OUTPUT);
  pinMode(pwmH2, OUTPUT);

  pinMode(dirH3, OUTPUT);
  pinMode(pwmH3, OUTPUT);

  pinMode(dirH4, OUTPUT);
  pinMode(pwmH4, OUTPUT);
}

// setup vertical thrusters
void setup_V_motors() {

  pinMode(pwmV1_1, OUTPUT);
  pinMode(pwmV1_2, OUTPUT);
  pinMode(enV1, OUTPUT);

  digitalWrite(enV1, HIGH);  // enable the bts

  pinMode(pwmV2_1, OUTPUT);
  pinMode(pwmV2_2, OUTPUT);
  pinMode(enV2, OUTPUT);

  digitalWrite(enV2, HIGH);  // enable the bts
}

// control horizontal motors
void controlHmotors() {
  for (int num = 0; num <= 3; num++) {
    digitalWrite(HorizontalThrusterPinsDir[num], (outputHorizontalThrusters[num] >= 0) ? HIGH : LOW);  // control direction
    analogWrite(HorizontalThrusterPinsSpeed[num], int(abs(outputHorizontalThrusters[num])));           // control speed
  }
}

// control vertical motors
void controlVmotors() {
  for (int num = 0; num <= 1; num++) {
    if (outputVerticalThrusters[num] >= 0) {
      analogWrite(VerticalThrusterSp1[num], int(abs(outputVerticalThrusters[num])));  // control speed
      analogWrite(VerticalThrusterSp2[num], 0);                                       // control speed
    } else {
      analogWrite(VerticalThrusterSp1[num], 0);                                       // control speed
      analogWrite(VerticalThrusterSp2[num], int(abs(outputVerticalThrusters[num])));  // control speed
    }
  }
}

void setupDCV() {
  pinMode(dcv1, OUTPUT);
  pinMode(dcv2, OUTPUT);
}

void controlMotors() {
  // control horizontal motors
  controlHmotors();
  // control vertical motors
  controlVmotors();
}

// Output thruster forces

// Apply constraints to the thruster forces
// if the maximum absolute force exceeds the max allowed force, scale down
void applyConstraints(float* thruster_forces, int size, float max_force) {
  float max_abs_force = 0;

  // Find the maximum absolute value of the thruster forces
  for (int i = 0; i < size; i++) {
    if (abs(thruster_forces[i]) > max_abs_force) {
      max_abs_force = abs(thruster_forces[i]);
    }
  }

  // If the maximum absolute force exceeds the max allowed force, scale down
  if (max_abs_force > max_force) {
    float scaling_factor = max_force / max_abs_force;
    for (int i = 0; i < size; i++) {
      thruster_forces[i] *= scaling_factor;  // Apply the scaling factor
    }
  }
}

// compute horizontal forces
void ComputeHorrizontalThrustForces(double* input, double T_inverse[4][3], float* outputThrusters) {

  // Perform matrix multiplication outputThrusters = T_inverse * input

  for (int i = 0; i < 4; i++) {
    outputThrusters[i] = 0;
    for (int j = 0; j < 3; j++) {
      outputThrusters[i] += T_inverse[i][j] * input[j];
    }
  }

  float max_force = 255;
  applyConstraints(outputThrusters, 4, max_force);
}

// compute vertical forces
void ComputeVerticalThrustForces(double* input, double T_inverse[2][2], float* outputThrusters) {

  // Perform matrix multiplication outputThrusters = T_inverse * input

  for (int i = 0; i < 2; i++) {
    outputThrusters[i] = 0;
    for (int j = 0; j < 2; j++) {
      outputThrusters[i] += T_inverse[i][j] * input[j];
    }
  }

  float max_force = 255;
  applyConstraints(outputThrusters, 2, max_force);
}

void calculateThrust(){
  // Compute the thruster forces for the horizontal thrusters
  ComputeHorrizontalThrustForces(inputH, T_inverse_Horizontal, outputHorizontalThrusters);

  // // Compute the thruster forces for the vertical thrusters
  ComputeVerticalThrustForces(inputV, T_inverse_Vertical, outputVerticalThrusters);
}

void readIncomingData() {
  if (Serial.available()) {
    last_time_data_received = millis();
    // Read the incoming data from the serial port
    Serial.readBytesUntil(terminator, incoming, incoming_data_length);

    // first check the checksum byte which is byte number 8
    // if the checksum is correct then read the data
    // if the checksum is not correct then ignore the data

    // Validate checksum (Byte 7 should be XOR of Bytes 0-6)
    byte xorCheck = 0;
    for (int i = 0; i <= 6; i++) {
      xorCheck ^= incoming[i];
    }

    if (incoming[7] != xorCheck) {
      Serial.println("Checksum error! Ignoring packet.");
      return;
    }

    // Read the input forces
    inputH[fx_index_inputH] = (float)incoming[fx_index_incoming_data] * (fx_max_input / fx_max_output) * ((incoming[sign_byte_index_incoming_data] & (1 << fx_index_incoming_data)) ? -1 : 1);       // Fx
    inputH[fy_index_inputH] = (float)incoming[fy_index_incoming_data] * (fy_max_input / fy_max_output) * ((incoming[sign_byte_index_incoming_data] & (1 << fy_index_incoming_data)) ? -1 : 1);       // Fy
    inputH[tau_index_inputH] = (float)incoming[tau_index_incoming_data] * (tau_max_input / tau_max_output) * ((incoming[sign_byte_index_incoming_data] & (1 << tau_index_incoming_data)) ? -1 : 1);  // Tau

    // Vertical forces
    inputV[fz_index_inputV] = (float)incoming[fz_index_incoming_data] * (fz_max_input / fz_max_output) * ((incoming[sign_byte_index_incoming_data] & (1 << fz_index_incoming_data)) ? -1 : 1);                      // Fz
    inputV[tpitch_index_inputV] = (float)incoming[tpitch_index_incoming_data] * (tpitch_max_input / tpitch_max_output) * ((incoming[sign_byte_index_incoming_data] & (1 << tpitch_index_incoming_data)) ? -1 : 1);  // Tpitch

    // Read the LED state
    ledState = incoming[led_index_incoming_data] & (1 << led_index_input);
    // Read the DC valve 1 state
    dcv1State = incoming[dcv1_index_incoming_data] & (1 << dcv1_index_input);
    // Read the DC valve 2 state
    dcv2State = incoming[dcv2_index_incoming_data] & (1 << dcv2_index_input);

    NULL_INPUT_YAW_FLAG = incoming[tau_index_incoming_data] ? 0 : 1;
    NULL_INPUT_PITCH_FLAG = incoming[tpitch_index_incoming_data] ? 0 : 1;
  }
}

void operatePID() {
  // Check if the input forces are zero for YAW
  // If the input forces are zero, start the PID controller for YAW
  // If the input forces are not zero, stop the PID controller for YAW
  if (NULL_INPUT_YAW_FLAG) {
    // if its the first time to start the PID controller, set the setpoint to the current yaw angle
    // if its not the first time, keep the setpoint as it is
    flag_YAW_PID ? (setpointYaw = setpointYaw) : (setpointYaw = yawAngle);
    // set the flag to true
    flag_YAW_PID = true;
    // set the input yaw angle
    inputYaw = yawAngle;
    // Start the PID controller for YAW
    PID_YAW(true);
    // set the output yaw torque
    inputH[tau_index_inputH] = outputYaw;
  } else {
    // Stop the PID controller for YAW
    PID_YAW(false);
    // set the flag to false
    flag_YAW_PID = false;
  }

  // Check if the input forces are zero for PITCH
  // If the input forces are zero, start the PID controller for PITCH
  // If the input forces are not zero, stop the PID controller for PITCH
  if (NULL_INPUT_PITCH_FLAG) {
    // if its the first time to start the PID controller, set the setpoint to the current pitch angle
    // if its not the first time, keep the setpoint as it is
    flag_PITCH_PID ? (setpointPitch = setpointPitch) : (setpointPitch = pitchAngle);
    // set the flag to true
    flag_PITCH_PID = true;
    // set the input pitch angle
    inputPitch = pitchAngle;
    // Start the PID controller for PITCH
    PID_PITCH(true);
    // set the output pitch torque
    inputV[tpitch_index_inputV] = outputPitch;
  } else {
    // Stop the PID controller for PITCH
    PID_PITCH(false);
    // set the flag to false
    flag_PITCH_PID = false;
  }
}

void dcv1Control(bool state) {
  // control the DC valve 1
  // if the state is true, open the valve
  // if the state is false, close the valve
  // DCV is just a DC motor
  digitalWrite(valvePins[0], state ? HIGH : LOW);
  Serial.print("DC Valve 1: ");
  Serial.println(state);
}

void dcv2Control(bool state) {
  // control the DC valve 2
  // if the state is true, open the valve
  // if the state is false, close the valve
  // DCV is just a DC motor
  digitalWrite(valvePins[1], state ? HIGH : LOW);
  Serial.print("DC Valve 2: ");
  Serial.println(state);
}

// IMU functions

void imu_read() {
  // Get Euler angles (in degrees)
  imu::Vector<3> myData = Bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yawAngle = myData.x();
  rollAngle = myData.y();
  pitchAngle = myData.z();
}

void debugThrusters() {
  Serial.println("Thruster Forces:");
  for (int i = 0; i < 4; i++) {
    Serial.print(" F");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(int(outputHorizontalThrusters[i]));  // Print with 2 decimal places
  }

  for (int i = 0; i < 2; i++) {
    Serial.print(" F");
    Serial.print(i + 5);
    Serial.print(": ");
    Serial.print(int(outputVerticalThrusters[i]));  // Print with 2 decimal places
  }

  Serial.println();
}
void debugSensors() {
  imu_read();
  Serial.print(yawAngle);
  Serial.print("  ");
  Serial.print(pitchAngle);
  Serial.print("  ");
  Serial.print(rollAngle);
  Serial.println();
}
void checkPitchPid(){
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    if (incoming == 's') {
      NULL_INPUT_PITCH_FLAG = 0;
    } else if (incoming == 'e') {
      NULL_INPUT_PITCH_FLAG = 1;
    }
  }
}

void checkYawPid(){
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    if (incoming == 's') {
      NULL_INPUT_YAW_FLAG = 0;
    } else if (incoming == 'e') {
      NULL_INPUT_YAW_FLAG = 1;
    }
  }
}
void setupBno() {
  // Initialize BNO055 sensor
  if (!Bno.begin()) {
    Serial.println("BNO055 not detected!");
  }
}

void stopMotors(){
  outputHorizontalThrusters[0] = 0;
  outputHorizontalThrusters[1] = 0;
  outputHorizontalThrusters[2] = 0;
  outputHorizontalThrusters[3] = 0;

  outputVerticalThrusters[0] = 0;
  outputVerticalThrusters[1] = 0;

  controlMotors();
}

void testMotors(){
  outputHorizontalThrusters[0] = 255;
  outputHorizontalThrusters[1] = 255;
  outputHorizontalThrusters[2] = 255;
  outputHorizontalThrusters[3] = 255;

  outputVerticalThrusters[0] = 255;
  outputVerticalThrusters[1] = 255;

  controlMotors();
}

void checkSerial(){
  if (millis() - last_time_data_received > serial_timeout_ms) {
    stopMotors();
    controlMotors();
  }
}

void checkAllSystem(){
  testMotors();
  delay(2000);
  stopMotors();
  delay(2000);
  turnLight(1);
  delay(2000);
  turnLight(0);
  delay(2000);
  dcv1Control(1);
  delay(2000);
  dcv1Control(0);
  delay(2000);
  dcv2Control(1);
  delay(2000);
  dcv2Control(0);
  delay(2000);
  debugSensors();
  Serial.println("All systems are working fine from Guileta");
}

void checkSerialDataAndControlMotors(){
  readIncomingData();
  imu_read();
  correctYawAngle();
  operatePID();
  calculateThrust();
  checkSerial();
  debugThrusters();
}

void sendSensorData() {
  /*
  All 6 thruster values (4 horizontal + 2 vertical)

  Orientation data (roll, pitch, yaw)

  Linear acceleration (x, y, z)

  System status (LED and DC valves states)

  Example output:
  {
  "thrusters":{"h1":125.5,"h2":130.2,"h3":-110.8,"h4":115.0,"v1":80.5,"v2":-90.2},
  "orientation":{"roll":1.25,"pitch":-0.75,"yaw":45.50},
  "acceleration":{"x":0.12,"y":-0.05,"z":9.78},
  "status":{"led":true,"dcv1":false,"dcv2":false}
  }
  */


  // Create a JSON-like string with all required data
  String data = "{";
  
  // Add thruster powers
  data += "\"thrusters\":{";
  data += "\"h1\":" + String(outputHorizontalThrusters[0], 1) + ",";
  data += "\"h2\":" + String(outputHorizontalThrusters[1], 1) + ",";
  data += "\"h3\":" + String(outputHorizontalThrusters[2], 1) + ",";
  data += "\"h4\":" + String(outputHorizontalThrusters[3], 1) + ",";
  data += "\"v1\":" + String(outputVerticalThrusters[0], 1) + ",";
  data += "\"v2\":" + String(outputVerticalThrusters[1], 1);
  data += "},";
  
  // Add orientation data
  data += "\"orientation\":{";
  data += "\"roll\":" + String(rollAngle, 2) + ",";
  data += "\"pitch\":" + String(pitchAngle, 2) + ",";
  data += "\"yaw\":" + String(yawAngle, 2);
  data += "},";
  
  // Add acceleration data
  imu::Vector<3> accel = Bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  data += "\"acceleration\":{";
  data += "\"x\":" + String(accel.x(), 2) + ",";
  data += "\"y\":" + String(accel.y(), 2) + ",";
  data += "\"z\":" + String(accel.z(), 2);
  data += "},";
  
  // Add system status
  data += "\"status\":{";
  data += "\"led\":" + String(ledState ? "true" : "false") + ",";
  data += "\"dcv1\":" + String(dcv1State ? "true" : "false") + ",";
  data += "\"dcv2\":" + String(dcv2State ? "true" : "false");
  data += "}";
  
  data += "}";
  
  // Send the data via Serial
  Serial.println(data);
}

void main(){
  // Read the incoming data from the serial port
  readIncomingData();

  // Read data from the IMU
  imu_read();

  // Correct the yaw angle to be between -180 and 180
  correctYawAngle();

  // PID controllers for YAW and PITCH
  operatePID();

  // Calculate the thruster forces
  calculateThrust();

  // Debug the thrusters
  #ifdef DEBUG_THRUSTERS
  debugThrusters();
  #endif

  // check that the serial is still working if not stop the motors
  checkSerial();

  // control the motors
  controlMotors();

  // Turn the light on or off
  turnLight(ledState);

  // Control the DC valve 1
  dcv1Control(dcv1State);

  // Control the DC valve 2
  dcv2Control(dcv2State);


   // Send sensor data to station
   sendSensorData();

}
/*
Masry add ur code here ( IMU Functions )
*/
// ########################################################### End of Functions ########################################################### //

void setup() {
  Serial.begin(115200);

  // setup motors
  setup_H_motors();
  setup_V_motors();

  pinMode(light, OUTPUT);

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

  main();
  /* delay(50); // not sure if this is correct but adding delay to help sending data smoothly over 20 khz (commented till expreminted) */ 

}