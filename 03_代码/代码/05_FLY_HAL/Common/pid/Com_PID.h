#ifndef __COM_PID_H
#define __COM_PID_H
#include "Com_Debug.h"
#include "Com_Config.h"

void Com_PID_Init(PID_Struct *pid, float integralMax, float outputMax, float deadband);
void Com_PID_ComputePID(PID_Struct *pid);
void Com_PID_CascadePID(PID_Struct *out, PID_Struct *in);

#endif
