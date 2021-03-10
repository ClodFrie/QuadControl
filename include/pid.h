#ifndef PID_H
#define PID_H

#include <stdio.h>

typedef struct {
    double prevTime;
    double prevValue;
    double intValue;
    double windup;
    double currentValue;

    double kP, kI, kD;
} PID;

void initPID(PID* pid, double kP, double kI, double kD, double windup, double prevTime) ;
void updatePID(PID* pid, double actTime, double currentValue);
void closePID(PID* pid);

#endif