#ifndef PID_H
#define PID_H

#include <stdio.h>

typedef struct {
    double prevTime;
    double prevInput;
    double intValue;
    double windup;
    double currentValue;

    double kP, kI, kD;
} PID;

void initPID(PID* pid, double kP, double kI, double kD, double windup, double prevTime);
void updatePID(PID* pid, double actTime, double input);
void updatePID_statespace(PID* pid, double actTime, double position, double speed);

#endif