#include "func.h"

// ########################################################### Functions ########################################################### //

// Lights control
void turnLight(bool state) {
  digitalWrite(led, state ? HIGH : LOW);
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
void applyConstraints(float *thruster_forces, int size, float max_force) {
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
void ComputeHorrizontalThrustForces(double *input, double T_inverse[4][3], float *outputThrusters) {

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
void ComputeVerticalThrustForces(double *input, double T_inverse[2][2], float *outputThrusters) {

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

void calculateThrust() {
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
    // handle if the difference between the setpoint and the input yaw angle is greater than 180 degrees
    if (abs(setpointYaw - inputYaw) > 180) {
      if (setpointYaw > inputYaw) {
        // e.g. setpoint = 170, input = -170 => difference = 340,
        //      fix => setpoint = 170 - 360 = -190, input = -170 => difference = 20
        setpointYaw -= 360;
      } else {
        // e.g. setpoint = -170, input = 170 => difference = 340,
        //      fix => setpoint = -170 + 360 = 190, input = 170 => difference = 20
        setpointYaw += 360;
      }
    }
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
  digitalWrite(dcv1, state ? HIGH : LOW);
  Serial.print("DC Valve 1: ");
  Serial.println(state);
}

void dcv2Control(bool state) {
  // control the DC valve 2
  // if the state is true, open the valve
  // if the state is false, close the valve
  // DCV is just a DC motor
  digitalWrite(dcv2, state ? HIGH : LOW);
  Serial.print("DC Valve 2: ");
  Serial.println(state);
}

// IMU functions

void setupBno() {
  // Initialize BNO055 sensor
  if (!Bno.begin()) {
    Serial.println("BNO055 not detected!");
  }
  Bno.setAxisRemap((Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)AXIS_REMAP_CONFIG);
  Bno.setAxisSign((Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)AXIS_REMAP_SIGN);
}

void imu_read() {
  // Get Euler angles (in degrees)
  imu::Vector<3> myData = Bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // Might need to remap the angles to match the ROV's orientation
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

void checkPitchPid() {
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    if (incoming == 's') {
      NULL_INPUT_PITCH_FLAG = 0;
    } else if (incoming == 'e') {
      NULL_INPUT_PITCH_FLAG = 1;
    }
  }
}

void checkYawPid() {
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    if (incoming == 'm') {
      NULL_INPUT_YAW_FLAG = 0;
    } else if (incoming == 'n') {
      NULL_INPUT_YAW_FLAG = 1;
    }
  }
}

void stopMotors() {
  outputHorizontalThrusters[0] = 0;
  outputHorizontalThrusters[1] = 0;
  outputHorizontalThrusters[2] = 0;
  outputHorizontalThrusters[3] = 0;

  outputVerticalThrusters[0] = 0;
  outputVerticalThrusters[1] = 0;

  controlMotors();
}

void testMotors() {
  outputHorizontalThrusters[0] = 255;
  outputHorizontalThrusters[1] = 255;
  outputHorizontalThrusters[2] = 255;
  outputHorizontalThrusters[3] = 255;

  outputVerticalThrusters[0] = 255;
  outputVerticalThrusters[1] = 255;

  controlMotors();
}

void checkSerial() {
  if (millis() - last_time_data_received > serial_timeout_ms) {
    stopMotors();
    controlMotors();
    Serial.print("No connection   ");
  }
}

void checkAllSystem() {
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

void checkSerialDataAndControlMotors() {
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

void mainC() {
  // Read the incoming data from the serial port
  readIncomingData();

  // Read data from the IMU
  imu_read();

  // Correct the yaw angle to be between -180 and 180
  correctYawAngle();

// if u need to debug PID for yaw just un comment in thedebug section
#ifdef DEBUG_PID_YAW
  checkYawPid();
#endif

// if u need to debug PID for pitch just un comment in thedebug section
#ifdef DEBUG_PID_PITCH
  checkPitchPid();
#endif

  // PID controllers for YAW and PITCH
  operatePID();

  // Calculate the thruster forces
  calculateThrust();

// Debug the thrusters
#ifdef DEBUG_THRUSTERS
  debugThrusters();
#endif

  //check that the serial is still working if not stop the motors
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

// ########################################################### End of Functions ########################################################### //
