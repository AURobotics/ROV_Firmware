#ifndef PID_H
#define PID_H

#include "define.h"

// ########################################################### PID Controllers ########################################################### //

// ############################################################## YAW PID ############################################################## //
extern float inputYaw, kpYaw, kiYaw, kdYaw, setpointYaw, outputYaw;
extern float maxOutputYaw, minOutputYaw;
extern bool flag_YAW_PID;

void PID_YAW(bool start_YAW_PID);

// ############################################################## PITCH PID ############################################################## //
extern float inputPitch, kpPitch, kiPitch, kdPitch, setpointPitch, outputPitch;
extern float maxOutputPitch, minOutputPitch;
extern bool flag_PITCH_PID;

void PID_PITCH(bool start_PITCH_PID);

// ########################################################### End of PID Controllers ########################################################### //

#endif // PID_H
