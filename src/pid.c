// Implement a PID Controller for UAV position KI_xy

#include "../include/pid.h"

#include <stdio.h>
#include <stdlib.h>

void initPID(PID* pid, double kP, double kI, double kD, double windup, double prevTime) {
    pid->prevTime = prevTime;
    pid->windup = windup;

    pid->kP = kP;
    pid->kI = kI;
    pid->kD = kD;

}
void updatePID(PID* pid, double actTime, double currentValue) {
    double windup = pid->windup;

    // calculate deltaT (Ta) --> because we have no real time system, this can change every loop iteration ;)
    double Ta = (actTime - pid->prevTime);  // Ta in ms

    // differential component
    double diff = (currentValue - pid->prevValue) / Ta;

    // integrate one time step
    double intValue = pid->currentValue + (pid->prevValue * Ta);

    // implement Anti-Windup
    if (windup > 0) {
        intValue = intValue > windup ? windup : intValue;
        intValue = intValue < -windup ? -windup : intValue;
    }
    // save prevValue;
    pid->prevValue = pid->currentValue;

    // calculate output
    pid->currentValue = currentValue * pid->kP + intValue * pid->kI + diff * pid->kP;

    // assign data to PID structure
    pid->currentValue = currentValue;
    pid->intValue = intValue;
    pid->prevTime = actTime;
    
    
}

void closePID(PID* pid) {
    free(pid);
}