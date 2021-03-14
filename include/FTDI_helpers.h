#ifndef FTDI_HELPERS_H
#define FTDI_HELPERS_H

#include <ftd2xx.h>

int openFTDI(FT_HANDLE* ftHandle);
int requestData(FT_HANDLE* ftHandle);
int sendCmd(FT_HANDLE* ftHandle);
int sendParams(FT_HANDLE* ftHandle);


#endif