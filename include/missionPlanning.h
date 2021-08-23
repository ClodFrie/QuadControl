#ifndef HOVERINGFLIGHT_H
#define HOVERINGFLIGHT_H

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/pid.h"
#include "quad.h"

int calculateHover(double I_safeX0, double I_safeY0, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, PID* pidz);

int lemniscateHover(double I_safeX0, double I_safeY0, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, PID* pidz);

int continousPath(double output[], double t, double t0, double a_max, double v_max, double s_d);

int pathMPC(double I_safeX0, double I_safeY0, struct QuadState* Quad, struct CONTROL* ctrl, double actTime);

int pathPID(double I_safeZ, double I_safeX, double I_safeY, struct QuadState* Quad, struct CONTROL* ctrl, double actTime, double Ft_i[4]);

int calculateIMUCHover(double actTime, double targetHeight, struct QuadState* Quad, double maxAngle, PID* pidx, PID* pidy, PID* pidz);

#endif