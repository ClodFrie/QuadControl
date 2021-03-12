#include <assert.h>
#include <ftd2xx.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>  // for bzero
#include <sys/time.h>
#include <time.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/crc.h"
#include "../include/hoveringFlight.h"
#include "../include/pid.h"
#include "../include/quad.h"
#include "../include/threads.h"

unsigned int crc0;

// function prototypes
int openFTDI(FT_HANDLE* ftHandle);
int requestData(FT_HANDLE* ftHandle);
int sendCmd(FT_HANDLE* ftHandle);
int sendParams(FT_HANDLE* ftHandle);
double get_time_ms();
void setParams(float kP_pos, float kD_pos, float kP_yaw, float kD_yaw, float kP_height, float kD_height);

enum STATES { IDLE,
              TAKEOFF,
              HOVER,
              RECTANGULAR_TRAJECTORY,
              LEMNISCATE_TRAJECTORY,
              IMUC_HOVER,
              IMUC_ONLINE_TRAJECTORY,
              LANDING } state;
// IMUC = IMU + Camera

PID pidx, pidy, pidz;

void* ioThread(void* vptr) {
    // cast pointer to QuadState type
    struct QuadState* Quadptr = (struct QuadState*)vptr;

    // create ftdi handle
    FT_HANDLE ftHandle = NULL;

    // define starting state as IDLE
    state = IDLE;

    // zero all variables
    bzero(&data, sizeof(data));
    bzero(&ctrl, sizeof(ctrl));
    bzero(&params, sizeof(params));

    crc0 = crc8(0, NULL, 0);

    // open FTDI device
    if (openFTDI(&ftHandle) < 0) {
        return NULL;
    }
    // initialize pid controller
    initPID(&pidx, 0.95, 0.175, 0.65, 2, get_time_ms());
    initPID(&pidy, 0.95, 0.205, 0.65, 2, get_time_ms());

    // initialize pid height controller
    initPID(&pidz, 2.8, 2, 2.3, 1.5, get_time_ms());

    // write control parameters
    // setParams(0, 0, 0, 0, 0, 0); // comparison to no controller
    setParams(0.007265, 0.008265, 0.004500, 0.0011250, 0, 0);
    while (sendParams(&ftHandle) != 0) {
        ;  // make sure that parameters have been received correctly
    }
    printf("[PARAM] received succesfully!\n");

    state = HOVER;
    while (1) {  // do forever
        double t1 = get_time_ms();
        // receive drone data
        requestData(&ftHandle);
        printf("BAT,%5d,CPU,%3d,yaw,%3d,", data.battery_voltage, data.HL_cpu_load, data.angle_yaw);
        // TODO: print data, make logfile, make graphs ,...

        if (pthread_mutex_lock(&state_mutex) == 0) {  // TODO: mutex could be released faster
            // calculate desired movements
            switch (state) {
                case HOVER:
                    calculateHover(0.6, -583.89, -77.12, 4, Quadptr, &ctrl, &pidx, &pidy, &pidz, get_time_ms());
                    break;
                default:
                    printf("illegal state\n");
                    break;
            }
            pthread_mutex_unlock(&state_mutex);

            // write control commands - only if new data has been calculated

            ctrl.CRC = crc8(crc0, (unsigned char*)(&ctrl), sizeof(ctrl) - 1);
            sendCmd(&ftHandle);
        }

        printf("[T],%3.2lf,", get_time_ms() - t1);
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

    ftStatus = FT_Write(*ftHandle, reqData, sizeof(reqData), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_Write returned %d\n", (int)ftStatus);
    }

    double endTime = get_time_ms() + MS_TIMEOUT;
    while (bytesReceived < sizeof(data)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(*ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            fprintf(stderr,"\nFailure.  FT_GetQueueStatus returned %d.\n",
                   (int)ftStatus);
            return 1;
        }

        // periodically check for time out!
        if (get_time_ms() > endTime) {
            fprintf(stderr,"[DATA] read timed out %lf \n", get_time_ms() - endTime);
            break;
        }
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(*ftHandle, &data, bytesReceived, &bytesRead);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }
    if (bytesRead != sizeof(data)) {  // incorrect data received
        bzero(&data, sizeof(data));   // delete corrupted data
        return 1;
    }

    unsigned char crc = crc8(crc0, (unsigned char*)(&data), sizeof(data) - 1);
    if (data.CRC != crc) {
        fprintf(stderr,"[CRC ERROR] Receiving side\n");
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
    char str[] = "noths recvd\n";
    char* answer;

    ftStatus = FT_Write(*ftHandle, sendParams, sizeof(sendParams), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_Write returned %d\n", (int)ftStatus);
    }
    ftStatus = FT_Write(*ftHandle, &params, sizeof(params), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_Write returned %d\n", (int)ftStatus);
    }

    double endTime = get_time_ms() + MS_TIMEOUT;
    while (bytesReceived < sizeof(str)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(*ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            fprintf(stderr,"\nFailure.  FT_GetQueueStatus returned %d.\n",
                   (int)ftStatus);
            return 1;
        }
        // periodically check for time out!
        if (get_time_ms() > endTime) {
            fprintf(stderr,"[PARAM] read timed out %lf \n", get_time_ms() - endTime);
            break;
        }
    }
    answer = (unsigned char*)calloc(sizeof(char), bytesReceived);
    if (answer == NULL) {
        fprintf(stderr,"[PARAM] malloc error");
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(*ftHandle, answer, bytesReceived, &bytesRead);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }
    if (bytesRead != sizeof(str)) {
        return 1;
    }
    if (bcmp(answer, "param recvd\n", sizeof(str)) != 0) {
        fprintf(stderr,"%s", answer);
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
    char str[] = "noths recvd\n";
    char* answer;

    ftStatus = FT_Write(*ftHandle, sendCmd, sizeof(sendCmd), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_Write returned %d\n", (int)ftStatus);
    }
    ftStatus = FT_Write(*ftHandle, &ctrl, sizeof(ctrl), &bytesWritten);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_Write returned %d\n", (int)ftStatus);
    }
    // printf("bytesWritten :%d\n",bytesWritten);
    double endTime = get_time_ms() + MS_TIMEOUT;

    while (bytesReceived < sizeof(str)) {  // while not all bytes were received
        ftStatus = FT_GetQueueStatus(*ftHandle, &bytesReceived);
        if (ftStatus != FT_OK) {
            fprintf(stderr,"\nFailure.  FT_GetQueueStatus returned %d.\n",
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
        fprintf(stderr,"[CMD] malloc error");
    }
    // Then copy D2XX's buffer to ours.
    ftStatus = FT_Read(*ftHandle, answer, bytesReceived, &bytesRead);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_Read returned %d.\n", (int)ftStatus);
        return 1;
    }
    if (bytesRead != sizeof(str)) {
        return 1;
    }
    if (bcmp(answer, "comms recvd\n", sizeof(str)) != 0) {
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
        fprintf(stderr,"FT_Open(%d) failed, with error %d.\n", portNum, (int)ftStatus);
        fprintf(stderr,"On Linux, lsmod can check if ftdi_sio (and usbserial) are present.\n");
        fprintf(stderr,"If so, unload them using rmmod, as they conflict with ftd2xx.\n");
        fprintf(stderr,"run unload script: \"sudo ./unload_drivers\"\n");
        return -1;
        ;
    }

    assert(*ftHandle != NULL);

    ftStatus = FT_GetDriverVersion(*ftHandle, (LPDWORD)(&driverVersion));
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_GetDriverVersion returned %d.\n",
               (int)ftStatus);
        return -1;
        ;
    }

    fprintf(stderr,"Using D2XX version %08x\n", driverVersion);

    ftStatus = FT_ResetDevice(*ftHandle);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_ResetDevice returned %d.\n", (int)ftStatus);
        return -1;
        ;
    }

    // Flow control is needed for higher baud rates
    ftStatus = FT_SetFlowControl(*ftHandle, FT_FLOW_RTS_CTS, 0, 0);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_SetFlowControl returned %d.\n", (int)ftStatus);
        return -1;
        ;
    }

    ftStatus = FT_SetDataCharacteristics(*ftHandle, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_SetDataCharacteristics returned %d.\n",
               (int)ftStatus);
        return -1;
        ;
    }

    ftStatus = FT_SetBaudRate(*ftHandle, baudRate);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_SetBaudRate(%d) returned %d.\n",
               (int)baudRate,
               (int)ftStatus);
        return -1;
        ;
    }

    // Assert Request-To-Send to prepare receiver
    ftStatus = FT_SetRts(*ftHandle);
    if (ftStatus != FT_OK) {
        fprintf(stderr,"Failure.  FT_SetRts returned %d.\n", (int)ftStatus);
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

void setParams(float kP_pos, float kD_pos, float kP_yaw, float kD_yaw, float kP_height, float kD_height) {
    params.kP_pos = kP_pos;
    params.kD_pos = kD_pos;

    params.kP_yaw = kP_yaw;
    params.kD_yaw = kD_yaw;

    params.kP_height = kP_height;
    params.kD_height = kD_height;
    params.CRC = crc8(crc0, (unsigned char*)(&params), sizeof(params) - 1);
}
