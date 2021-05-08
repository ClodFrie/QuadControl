#include <ftd2xx.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>  // for bzero

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/FTDI_helpers.h"
#include "../include/crc.h"
#include "../include/hoveringFlight.h"
#include "../include/kalman.h"
#include "../include/pid.h"
#include "../include/quad.h"
#include "../include/threads.h"
#include "../include/time_helper.h"

unsigned int crc0;

// function prototypes
FILE* initLogFile(char* mType);
int writeLogLine(FILE* fd, double deltaT, short bat, short cpu, short yaw, struct QuadState* Quadptr);

double get_time_ms();
void setParams(float kP_pos, float kD_pos, float kP_yaw, float kD_yaw, float kP_height, float kD_height);
int updateState(struct QuadState* Quad);

enum STATES { IDLE,
              TAKEOFF,
              HOVER,
              RECTANGULAR_TRAJECTORY,
              LEMNISCATE_TRAJECTORY,
              IMUC_HOVER,
              IMUC_ONLINE_TRAJECTORY,
              LANDING } state;
// IMUC = IMU + Camera

PID pidx, pidy, pidz;  // create pid variables

void* ioThread(void* vptr) {
    // cast pointer to QuadState type
    struct QuadState* Quadptr = (struct QuadState*)vptr;

    // create ftdi handle
    FT_HANDLE ftHandle = NULL;

    // initialize log file
    FILE* fd = initLogFile("IDLE.X");

    // define starting state as IDLE
    state = IDLE;

    // initialize crc0
    unsigned char crc0 = crc8(0, NULL, 0);  // TODO: move this somewhere more intelligent

    // zero all variables
    bzero(&data, sizeof(data));
    bzero(&ctrl, sizeof(ctrl));
    bzero(&params, sizeof(params));

    // open FTDI device
    if (openFTDI(&ftHandle) < 0) {
        return NULL;
    }

    // initialize pid controllers
    // TODO: redone PID Controller: check before use
    // Ziegler Nichols:
    double Pkrit, Tkrit, Tn, Tv, Kp, TI, TD;
    Pkrit = 5.65;
    Kp = 0.6 * Pkrit;
    Tkrit = 4.61;
    Tn = 0.5 * Tkrit;
    Tv = 0.125 * Tkrit;
    TD = Tv * Kp;
    TI = Tn / Kp;
    double Kd = TD;
    double Ki = 1.0 / TI;
    // END Ziegler Nichols

    // initPID(&pidx, 0.35, 0.20, 1.65, 8.50, get_time_ms());  // pidX
    // initPID(&pidy, 0.35, 0.19, 1.56, 8.50, get_time_ms());  // pidY
    // initPID(&pidz, 2.90, 2.25, 3.55, 10, get_time_ms());    // pidZ -- TODO: still not properly tuned

    // test parameters
    initPID(&pidx, 0.15, 0.45, 3.25, 1.0 / 0.16, get_time_ms());  // pidX
    initPID(&pidy, 0.19, 0.47, 3.25, 1.0 / 0.16, get_time_ms());  // pidY
    initPID(&pidz, 10.50, 8.5, 40.55, 10, get_time_ms());        // pidZ -- TODO: still not properly tuned

    initKalman();

    // write control parameters
    setParams(0.007265, 0.008265 + 0.002, 0.004500, 0.0011250, 0, 0);
    while (sendParams(&ftHandle) != 0) {
        ;  // make sure that parameters have been received
    }
    printf("[PARAM] received succesfully!\n");

    state = HOVER;
    while (1) {  // do forever
        double t1 = get_time_ms();
        // receive drone data
        requestData(&ftHandle);
        printf("BAT,%5d,CPU,%3d,yaw,%3d,", data.battery_voltage, data.HL_cpu_load, data.angle_yaw);

        if (pthread_mutex_lock(&state_mutex) == 0) {
            updateState(Quadptr);

            // release mutex
            pthread_mutex_unlock(&state_mutex);

            // calculate desired movements
            switch (state) {
                case IDLE:
                    break;
                case TAKEOFF:
                    break;
                case HOVER:
                    calculateHover(600, -1878.92, 705.49, 4, Quadptr, &ctrl, &pidx, &pidy, &pidz, get_time_ms());
                    break;
                case RECTANGULAR_TRAJECTORY:
                    break;
                default:
                    printf("illegal state\n");
                    break;
            }

            // write control commands - only if new data has been generated
            ctrl.CRC = crc8(crc0, (unsigned char*)(&ctrl), sizeof(ctrl) - 1);
            sendCmd(&ftHandle);

            // write logfile
            writeLogLine(fd, get_time_ms() - t1, data.battery_voltage, data.HL_cpu_load, data.angle_yaw, Quadptr);
        }
        printf("[T],%3.2lf,", get_time_ms() - t1);
    }

    // free resources
    if (ftHandle != NULL) {
        FT_Close(ftHandle);
        fclose(fd);
    }
    return NULL;
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

FILE* initLogFile(char* mType) {
    // build file name from current time and mission type
    char buffTime[100];
    char buffFileName[1024];
    get_date(buffTime);
    sprintf(buffFileName, "../../Flight_Data/%s_%s.csv", mType, buffTime);
    printf("%s\n", buffFileName);

    // open file for writing and get file descriptor
    FILE* fd = fopen(buffFileName, "w+");

    // not able to open, return NULL
    if (fd == NULL) {
        return NULL;
    }

    // write first line (header)
    fprintf(fd, "T,dT,BAT,CPU,YAW,U_THRUST,I_X,I_Y,I_Z,ROLL_CMD,PITCH_CMD,YAW_CMD\n");

    // return file descriptor for use in other functions
    return fd;
}
// write current data for every time step
int writeLogLine(FILE* fd, double deltaT, short bat, short cpu, short yaw, struct QuadState* Quadptr) {
    fprintf(fd, "%lf,%lf,%d,%d,%d,%u,%lf,%lf,%lf,%d,%d,%d\n", get_time_ms(), deltaT, data.battery_voltage, data.HL_cpu_load, data.angle_yaw,
            ctrl.u_thrust, Quadptr->I_x, Quadptr->I_y, Quadptr->I_z, ctrl.roll_d, ctrl.pitch_d, ctrl.yaw_d);

    fflush(fd);  // flush file buffer, so that every line is written and not lost when aborting execution [CTRL]+[C]}
}

int updateState(struct QuadState* Quad) {
    // START critical section
    double q4 = Quad->roll;
    double q5 = Quad->pitch;
    double q6 = Quad->yaw;

    double Qx0 = Quad->Qx;
    double Qy0 = Quad->Qy;
    double Qz0 = Quad->Qz;

    Quad->I_x = cos(q5) * cos(q6) * Qx0 + (sin(q4) * sin(q5) * cos(q6) - cos(q4) * sin(q6)) * Qy0 + (cos(q4) * sin(q5) * cos(q6) + sin(q4) * sin(q6)) * Qz0;
    Quad->I_y = cos(q5) * sin(q6) * Qx0 + (sin(q4) * sin(q5) * sin(q6) + cos(q4) * cos(q6)) * Qy0 + (cos(q4) * sin(q5) * sin(q6) - sin(q4) * cos(q6)) * Qz0;
    Quad->I_z = -sin(q5) * Qx0 + sin(q4) * cos(q5) * Qy0 + cos(q4) * cos(q5) * Qz0;

    // state estimation: kalman filter
    double states[6];
    kalmanFilter(states, Quad->I_x, Quad->I_y, Quad->I_z);

    // TODO: test before use
    Quad->I_x = states[0];
    Quad->I_x_dot = states[1];
    Quad->I_y = states[2];
    Quad->I_y_dot = states[3];
    Quad->I_z = states[4];
    Quad->I_z_dot = states[5];

    // END critical section
    return 0;
}