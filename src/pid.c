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

    pid->currentValue = 0;
    pid->intValue = 0;
    pid->prevInput = 0;
}
void updatePID(PID* pid, double actTime, double input) {
    // calculate deltaT (Ta) --> because we have no real time system, this can change every loop iteration ;)
    double Ta = (actTime - pid->prevTime) / 1000.0;  // Ta in s

    // differential component
    double diff = (input - pid->prevInput) / Ta;

    // integrate one time step
    double intValue = pid->intValue + (pid->prevInput * Ta);  // maybe a different method for integration could be used

    // implement Anti-Windup
    double windup = pid->windup;
    if (windup > 0) {
        intValue = intValue > windup ? windup : intValue;
        intValue = intValue < -windup ? -windup : intValue;
    }

    // assign data to PID structure
    pid->intValue = intValue;
    pid->prevTime = actTime;
    pid->prevInput = input;  // save prevValue;

    // calculate output
    pid->currentValue = input * pid->kP + intValue * pid->kI + diff * pid->kD;
}

void updatePID_statespace(PID* pid, double actTime, double position, double speed) {
    // calculate deltaT (Ta) --> because we have no real time system, this can change every loop iteration ;)
    double Ta = (actTime - pid->prevTime) / 1000.0;  // Ta in s

    // integrate one time step
    double intValue = pid->intValue + (pid->prevInput * Ta);

    // implement Anti-Windup
    double windup = pid->windup;
    if (windup > 0) {
        intValue = intValue > windup ? windup : intValue;
        intValue = intValue < -windup ? -windup : intValue;
    }

    // assign data to PID structure
    pid->intValue = intValue;
    pid->prevTime = actTime;
    pid->prevInput = position;  // save prevValue;

    // calculate output
    pid->currentValue = position * pid->kP + intValue * pid->kI + speed * pid->kD;
}