#ifndef __COM_PID_H
#define __COM_PID_H
#include "Com_Debug.h"
#include "Com_Config.h"
void Com_PID_ComputePID(PID_Struct *pid);

void Com_PID_CascadePID(PID_Struct *out, PID_Struct *in);

#endif 

