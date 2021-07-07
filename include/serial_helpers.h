#ifndef SERIAL_HELPERS_H
#define SERIAL_HELPERS_H

#include <stdio.h>

int openPort(char portNo);
int sendCommand(int fd);
int requestData_ser(int fd);
int sendParameters(int fd);

#endif