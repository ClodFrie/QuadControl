#include <assert.h>
#include <ftd2xx.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>  // for bzero
#include <sys/time.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/crc.h"
#include "../include/hoveringFlight.h"
#include "../include/quad.h"
#include "../include/threads.h"

// function prototypes
int openFTDI(FT_HANDLE* ftHandle);
int requestData(FT_HANDLE* ftHandle);
int sendCmd(FT_HANDLE* ftHandle);
int sendParams(FT_HANDLE* ftHandle);
double get_time_ms();

void* ioThread(void* vptr) {
    FT_HANDLE ftHandle = NULL;

    // zero all variables
    bzero(&data, sizeof(data));
    bzero(&ctrl, sizeof(ctrl));
    bzero(&params, sizeof(params));

    // open FTDI device
    if (openFTDI(&ftHandle) < 0) {
        return NULL;
    }

    // write control parameters
    sendParams(&ftHandle);

    if (1) {  // do forever
        // receive drone data
        requestData(&ftHandle);
        
        if (pthread_mutex_trylock(&state_mutex) == 0) {
            // calculate desired movements
            calculateHover(0.5, &Quad, &ctrl);
            pthread_mutex_unlock(&state_mutex);

            // write control commands - only if new data has been calculated
            sendCmd(&ftHandle);
        }
        
    }

    // free ressources
    if (ftHandle != NULL)
        FT_Close(ftHandle);

    return NULL;
}
int requestData(FT_HANDLE* ftHandle) {
    FT_STATUS ftStatus;
    char reqData[] = {'>', '*', '>', 'd'};
    DWORD bytesWritten = 0;
    DWORD bytesReceived = 0;
    DWORD bytesRead = 0;
    double t1 = get_time_ms();

    ftStatus = FT_Write(ftHandle, reqData, sizeof(reqData), &bytesWritten);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_Write returned %d\n", (int)ftStatus);
    }

    printf("%d bytes written.\n", (int)bytesWritten);

    double endTime = get_time_ms() + MS_TIMEOUT;
    while (bytesReceived < sizeof(data)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            printf("\nFailure.  FT_GetQueueStatus returned %d.\n",
                   (int)ftStatus);
            return 1;
        }

        // periodically check for time out!
        if (get_time_ms() > endTime) {
            printf("read timed out \n");
            return 1;  // TODO: remove this statement
            break;
        }
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(ftHandle, &data, sizeof(data), &bytesRead);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }

    unsigned int crc = crc8(0, NULL, 0);
    crc = crc8(crc, (unsigned char*)(&data), sizeof(data) - 1);
    if (data.CRC == crc) {
        printf("data: %d\n", data.test);
    } else {
        printf("[CRC ERROR]\n");
    }

    printf("%f\n", get_time_ms() - t1);
    return 0;
}

int sendParams(FT_HANDLE* ftHandle) {
    FT_STATUS ftStatus;
    char sendParams[] = {'>', '*', '>', 'p'};
    DWORD bytesWritten = 0;
    DWORD bytesReceived = 0;
    DWORD bytesRead = 0;
    char answer[] = "noths recvd\n";
    double t1 = get_time_ms();

    ftStatus = FT_Write(ftHandle, sendParams, sizeof(sendParams), &bytesWritten);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_Write returned %d\n", (int)ftStatus);
    }

    printf("%d bytes written.\n", (int)bytesWritten);

    double endTime = get_time_ms() + MS_TIMEOUT;
    while (bytesReceived < sizeof(answer)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            printf("\nFailure.  FT_GetQueueStatus returned %d.\n",
                   (int)ftStatus);
            return 1;
        }

        // periodically check for time out!
        if (get_time_ms() > endTime) {
            printf("read timed out \n");
            return 1;  // TODO: remove this statement
            break;
        }
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(ftHandle, &answer, sizeof(answer), &bytesRead);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }

    printf("%f\n", get_time_ms() - t1);
    return 0;
}

int sendCmd(FT_HANDLE* ftHandle) {
    FT_STATUS ftStatus;
    char sendCmd[] = {'>', '*', '>', 'c'};
    DWORD bytesWritten = 0;
    DWORD bytesReceived = 0;
    DWORD bytesRead = 0;
    char answer[] = "noths recvd\n";
    double t1 = get_time_ms();

    ftStatus = FT_Write(ftHandle, sendCmd, sizeof(sendCmd), &bytesWritten);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_Write returned %d\n", (int)ftStatus);
    }

    printf("%d bytes written.\n", (int)bytesWritten);

    double endTime = get_time_ms() + MS_TIMEOUT;
    while (bytesReceived < sizeof(answer)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            printf("\nFailure.  FT_GetQueueStatus returned %d.\n",
                   (int)ftStatus);
            return 1;
        }
        // periodically check for time out!
        if (get_time_ms() > endTime) {
            printf("read timed out \n");
            return 1;  // TODO: remove this statement
            break;
        }
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(ftHandle, &answer, sizeof(answer), &bytesRead);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }

    printf("%f\n", get_time_ms() - t1);
    return 0;
}

int openFTDI(FT_HANDLE* ftHandle) {
    // source: FTDI Examples
    int portNum = 0;
    int driverVersion = 0;
    int baudRate = 57600;  // fixed for device - could theoretically be set higher
    FT_STATUS ftStatus;

    // unload linux drivers
    printf("Unload linux drivers\n");
    system("sudo rmmod ftdi_sio");
    system("sudo rmmod usbserial");

    printf("Opening FTDI device %d.\n", portNum);
    ftStatus = FT_Open(portNum, ftHandle);
    if (ftStatus != FT_OK) {
        printf("FT_Open(%d) failed, with error %d.\n", portNum, (int)ftStatus);
        printf("On Linux, lsmod can check if ftdi_sio (and usbserial) are present.\n");
        printf("If so, unload them using rmmod, as they conflict with ftd2xx.\n");
        return -1;
        ;
    }

    assert(*ftHandle != NULL);

    ftStatus = FT_GetDriverVersion(*ftHandle, (LPDWORD)(&driverVersion));
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_GetDriverVersion returned %d.\n",
               (int)ftStatus);
        return -1;
        ;
    }

    printf("Using D2XX version %08x\n", driverVersion);

    ftStatus = FT_ResetDevice(*ftHandle);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_ResetDevice returned %d.\n", (int)ftStatus);
        return -1;
        ;
    }

    // Flow control is needed for higher baud rates
    ftStatus = FT_SetFlowControl(*ftHandle, FT_FLOW_RTS_CTS, 0, 0);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_SetFlowControl returned %d.\n", (int)ftStatus);
        return -1;
        ;
    }

    ftStatus = FT_SetDataCharacteristics(*ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_SetDataCharacteristics returned %d.\n",
               (int)ftStatus);
        return -1;
        ;
    }

    ftStatus = FT_SetBaudRate(*ftHandle, baudRate);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_SetBaudRate(%d) returned %d.\n",
               (int)baudRate,
               (int)ftStatus);
        return -1;
        ;
    }

    // Assert Request-To-Send to prepare receiver
    ftStatus = FT_SetRts(*ftHandle);
    if (ftStatus != FT_OK) {
        printf("Failure.  FT_SetRts returned %d.\n", (int)ftStatus);
        return -1;
        ;
    }
    return 0;
}
double get_time_ms() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec + (t.tv_usec / 1000000.0)) * 1000.0;
}
