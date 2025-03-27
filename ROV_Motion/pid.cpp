#include "pid.h"

// ############################################################## YAW PID ############################################################## //
float inputYaw = 0, kpYaw = 0.7, kiYaw = 0.0, kdYaw = 0.1, setpointYaw = 0, outputYaw = 0;
float maxOutputYaw = 173.4, minOutputYaw = -173.4;
bool flag_YAW_PID = false;

void PID_YAW(bool start_YAW_PID) {
  double error, derror, dt;
  static float ierror = 0;
  static double prvError = 0;
  static unsigned long prvMillis = 0;

  if (start_YAW_PID) {
    unsigned long currentMillis = millis();
    if (currentMillis - prvMillis >= 10) {
      dt = (currentMillis - prvMillis) / 1000.0;
      prvMillis = currentMillis;
      
      error = (double)(setpointYaw - inputYaw);
      derror = (error - prvError) / dt;
      prvError = error;

      if (outputYaw < maxOutputYaw && outputYaw > minOutputYaw) {
        ierror += error * dt;
      }

      outputYaw = kpYaw * error + kiYaw * ierror + kdYaw * derror;
      outputYaw = constrain(outputYaw, minOutputYaw, maxOutputYaw);

#ifdef DEBUG_PID_YAW
      Serial.print("dt : "); Serial.print(dt);
      Serial.print(" Error : "); Serial.print(error);
      Serial.print(" dError : "); Serial.print(derror);
      Serial.print(" prvError : "); Serial.print(prvError);
      Serial.print(" setpoint : "); Serial.print(setpointYaw);
      Serial.print(" Yaw input : "); Serial.print(inputYaw);
      Serial.print(" Yaw output : "); Serial.println(outputYaw);
#endif
    }
  } else {
    ierror = 0;
    prvError = 0;
    outputYaw = 0;
  }
}

// ############################################################## PITCH PID ############################################################## //
float inputPitch = 0, kpPitch = 1.1, kiPitch = 0, kdPitch = 0, setpointPitch = 0, outputPitch = 0;
float maxOutputPitch = 102, minOutputPitch = -102;
bool flag_PITCH_PID = false;

void PID_PITCH(bool start_PITCH_PID) {
  double error, derror, dt;
  static float ierror = 0;
  static double prvError;
  static unsigned long prvMillis = 0;
  unsigned long currentMillis = millis();

  if (start_PITCH_PID) {
    if (currentMillis - prvMillis >= 10) {
      dt = (currentMillis - prvMillis) / 1000.0;
      prvMillis = millis();
      
      error = setpointPitch - inputPitch;
      derror = (error - prvError) / dt;
      prvError = error;

      if (outputPitch < maxOutputPitch && outputPitch > minOutputPitch) {
        ierror += error * dt;
      }

      outputPitch = kpPitch * error + kiPitch * ierror + kdPitch * derror;
      outputPitch = constrain(outputPitch, minOutputPitch, maxOutputPitch);

#ifdef DEBUG_PID_PITCH
      Serial.print("dt : "); Serial.print(dt);
      Serial.print(" Error : "); Serial.print(error);
      Serial.print(" dError : "); Serial.print(derror);
      Serial.print(" prvError : "); Serial.print(prvError);
      Serial.print(" setpoint : "); Serial.print(setpointPitch);
      Serial.print(" Pitch input : "); Serial.print(inputPitch);
      Serial.print(" Pitch output : "); Serial.println(outputPitch);
#endif
    }
  } else {
    ierror = 0;
    prvError = 0;
    outputPitch = 0;
  }
}
