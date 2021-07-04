#include "../include/FTDI_helpers.h"

#include <assert.h>
#include <ftd2xx.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/crc.h"
#include "../include/time_helper.h"

int requestData(FT_HANDLE* ftHandle) {
    FT_STATUS ftStatus;
    char reqData[] = {'>', '*', '>', 'd'};
    DWORD bytesWritten = 0;
    DWORD bytesReceived = 0;
    DWORD bytesRead = 0;

    ftStatus = FT_Write(*ftHandle, reqData, sizeof(reqData), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_Write returned %d\n", (int)ftStatus);
    }

    double endTime = get_time_ms() + MS_TIMEOUT;
    while (bytesReceived < sizeof(data)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(*ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            fprintf(stderr, "\nFailure.  FT_GetQueueStatus returned %d.\n",
                    (int)ftStatus);
            return 1;
        }

        // periodically check for time out!
        if (get_time_ms() > endTime) {
            fprintf(stderr, "[DATA] read timed out %lf \n", get_time_ms() - endTime);
            break;
        }
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(*ftHandle, &data, bytesReceived, &bytesRead);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }
    if (bytesRead != sizeof(data)) {  // incorrect data received
        bzero(&data, sizeof(data));   // delete corrupted data
        return 1;
    }
    unsigned char crc0 = crc8(0, NULL, 0);
    unsigned char crc = crc8(crc0, (unsigned char*)(&data), sizeof(data) - 1);
    if (data.CRC != crc) {
        fprintf(stderr, "[CRC ERROR] Receiving side\n");
        bzero(&data, sizeof(data));  // delete corrupted data
    }
    return 0;
}

int sendParams(FT_HANDLE* ftHandle) {
    FT_STATUS ftStatus;
    char sendParams[] = {'>', '*', '>', 'p'};
    DWORD bytesWritten = 0;
    DWORD bytesReceived = 0;
    DWORD bytesRead = 0;
    char str[] = "NK";
    char* answer;

    ftStatus = FT_Write(*ftHandle, sendParams, sizeof(sendParams), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_Write returned %d\n", (int)ftStatus);
    }
    ftStatus = FT_Write(*ftHandle, &params, sizeof(params), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_Write returned %d\n", (int)ftStatus);
    }

    double endTime = get_time_ms() + MS_TIMEOUT;
    while (bytesReceived < sizeof(str)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(*ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            fprintf(stderr, "\nFailure.  FT_GetQueueStatus returned %d.\n",
                    (int)ftStatus);
            return 1;
        }
        // periodically check for time out!
        if (get_time_ms() > endTime) {
            fprintf(stderr, "[PARAM] read timed out %lf \n", get_time_ms() - endTime);
            break;
        }
    }
    answer = (unsigned char*)calloc(sizeof(char), bytesReceived);
    if (answer == NULL) {
        fprintf(stderr, "[PARAM] malloc error");
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(*ftHandle, answer, bytesReceived, &bytesRead);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }
    if (bytesRead != sizeof(str)) {
        return 1;
    }
    if (bcmp(answer, "OK", (int)bytesReceived) != 0) {
        fprintf(stderr, "%s", answer);
        return 1;
    }
    free(answer);

    return 0;
}

int sendCmd(FT_HANDLE* ftHandle) {
    FT_STATUS ftStatus;
    char sendCmd[] = {'>', '*', '>', 'c'};
    DWORD bytesWritten = 0;
    DWORD bytesReceived = 0;
    DWORD bytesRead = 0;
    char str[] = "NK";
    char* answer;

    ftStatus = FT_Write(*ftHandle, sendCmd, sizeof(sendCmd), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_Write returned %d\n", (int)ftStatus);
    }
    ftStatus = FT_Write(*ftHandle, &ctrl, sizeof(ctrl), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_Write returned %d\n", (int)ftStatus);
    }
    // printf("bytesWritten :%d\n",bytesWritten);
    double endTime = get_time_ms() + MS_TIMEOUT;

    while (bytesReceived < sizeof(str)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(*ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            fprintf(stderr, "\nFailure.  FT_GetQueueStatus returned %d.\n",
                    (int)ftStatus);
            return 1;
        }
        // periodically check for time out!
        if (get_time_ms() > endTime) {
            printf("[CMD] read timed out \n");
            break;
        }
    }
    answer = (unsigned char*)calloc(sizeof(char), bytesReceived);
    if (answer == NULL) {
        fprintf(stderr, "[CMD] malloc error");
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(*ftHandle, answer, bytesReceived, &bytesRead);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }
    if (bytesRead != sizeof(str)) {
        return 1;
    }
    if (bcmp(answer, "OK", (int)bytesReceived) != 0) {
        printf("%s", answer);
        return 1;
    }

    free(answer);
    return 0;
}

int openFTDI(FT_HANDLE* ftHandle) {
    // source: FTDI Examples
    int portNum = 0;
    int driverVersion = 0;
    int baudRate = 57600;  // fixed for device - could theoretically be set higher
    FT_STATUS ftStatus;

    printf("Opening FTDI device %d.\n", portNum);
    ftStatus = FT_Open(portNum, ftHandle);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "FT_Open(%d) failed, with error %d.\n", portNum, (int)ftStatus);
        fprintf(stderr, "On Linux, lsmod can check if ftdi_sio (and usbserial) are present.\n");
        fprintf(stderr, "If so, unload them using rmmod, as they conflict with ftd2xx.\n");
        fprintf(stderr, "run unload script: \"sudo ./unload_drivers\"\n");
        return -1;
        ;
    }

    assert(*ftHandle != NULL);

    ftStatus = FT_GetDriverVersion(*ftHandle, (LPDWORD)(&driverVersion));
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_GetDriverVersion returned %d.\n",
                (int)ftStatus);
        return -1;
        ;
    }

    fprintf(stderr, "Using D2XX version %08x\n", driverVersion);

    ftStatus = FT_ResetDevice(*ftHandle);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_ResetDevice returned %d.\n", (int)ftStatus);
        return -1;
        ;
    }

    // Flow control is needed for higher baud rates
    ftStatus = FT_SetFlowControl(*ftHandle, FT_FLOW_RTS_CTS, 0, 0);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_SetFlowControl returned %d.\n", (int)ftStatus);
        return -1;
        ;
    }

    ftStatus = FT_SetDataCharacteristics(*ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_SetDataCharacteristics returned %d.\n",
                (int)ftStatus);
        return -1;
        ;
    }

    ftStatus = FT_SetBaudRate(*ftHandle, baudRate);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_SetBaudRate(%d) returned %d.\n",
                (int)baudRate,
                (int)ftStatus);
        return -1;
        ;
    }

    // Assert Request-To-Send to prepare receiver
    ftStatus = FT_SetRts(*ftHandle);
    if (ftStatus != FT_OK) {
        fprintf(stderr, "Failure.  FT_SetRts returned %d.\n", (int)ftStatus);
        return -1;
        ;
    }
    return 0;
}