#ifndef HOVERINGFLIGHT_H
#define HOVERINGFLIGHT_H

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/pid.h"
#include "quad.h"

int calculateHover(double* height, double I_safeX0,double* I_safeX, double* I_safeY, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, PID* pidz, double actTime);

int continousPath(double output[], double t, double t0, double a_max, double v_max, double s_d);

int pathMPC(double Q_safeZ, double Q_safeX, double Q_safeY, double I_safeZ, double I_safeX, double I_safeY, struct QuadState* Quad, struct CONTROL* ctrl, double actTime, double Ft_i[4]);

int pathPID(double I_safeZ, double I_safeX, double I_safeY, struct QuadState* Quad, struct CONTROL* ctrl, double actTime, double Ft_i[4]);

#endif