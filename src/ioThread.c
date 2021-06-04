#include <ftd2xx.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>  // for bzero

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/FTDI_helpers.h"
#include "../include/crc.h"
#include "../include/kalman.h"
#include "../include/missionPlanning.h"
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
void plotPath();

enum STATES {
    INIT,
    IDLE,
    TAKEOFF,
    HOVER,
    RECTANGULAR_TRAJECTORY,
    LEMNISCATE_TRAJECTORY,
    IMUC_HOVER,
    IMUC_ONLINE_TRAJECTORY,
    LANDING
} state;
// IMUC = IMU + Camera

PID pidx, pidy, pidz;  // create pid variables

void* ioThread(void* vptr) {
    // cast pointer to QuadState type
    struct QuadState* Quadptr = (struct QuadState*)vptr;

    // define starting state as INIT
    state = INIT;

    // create ftdi handle
    FT_HANDLE ftHandle = NULL;

    // initialize log file
    FILE* fd = initLogFile("IDLE.X");

    // open FTDI device
    if (openFTDI(&ftHandle) < 0) {
        return NULL;
    }

    // zero all variables
    bzero(&data, sizeof(data));
    bzero(&ctrl, sizeof(ctrl));
    bzero(&params, sizeof(params));

    // initialize crc0
    unsigned char crc0 = crc8(0, NULL, 0);  // TODO: move this somewhere more intelligent

    // initialize kalman filter
    initKalman();  // TODO: not working, why??

    // TODO: implement start trajectory logic
    // int startX = 0;
    // int startY = 0;

    double t1 = get_time_ms();
    // write control parameters
    setParams(0.007265, 0.008265 + 0.002, 0.004500, 0.0011250, 0, 0);
    while (sendParams(&ftHandle) != 0) {
        ;  // make sure that parameters have been received
    }
    printf("[PARAM] received succesfully!\n");

    // plotPath(t1);  // gnuplot

    double safeX = -1878.92;
    double safeY = 705.49;
    while (1) {  // do forever

        // receive drone data
        requestData(&ftHandle);
        printf("BAT,%5d,CPU,%3d,yaw,%3d,", data.battery_voltage, data.HL_cpu_load, data.angle_yaw);

        if (pthread_mutex_lock(&state_mutex) == 0) {
            updateState(Quadptr);

            // release mutex
            pthread_mutex_unlock(&state_mutex);

            // calculate desired movements
            printf("st: %d,", state);
            switch (state) {
                case INIT:  // wait for measurements from Qualisys system
                    if (Quadptr->I_z >= 0) {
                        // safeX = Quadptr->I_x;
                        // safeY = Quadptr->I_y;    

                        state = IDLE;  // change active state to idle
                    }
                    break;
                case IDLE:  // wait for some kind of start signal

                    // currently there is no idle state
                    state = TAKEOFF;

                    break;
                case TAKEOFF:
                    // currently there is no takeoff state
                    // state = HOVER;

                    pathMPC(0.6,safeX,safeY,Quadptr,&ctrl,get_time_ms());

                    break;
                case HOVER:
                    
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
        t1 = get_time_ms();
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
    fprintf(fd, "T,dT,BAT,CPU,YAW,U_THRUST,I_X,I_Y,I_Z,ROLL_CMD,PITCH_CMD,YAW_CMD,ROLL,PITCH\n");

    // return file descriptor for use in other functions
    return fd;
}
// write current data for every time step
int writeLogLine(FILE* fd, double deltaT, short bat, short cpu, short yaw, struct QuadState* Quadptr) {
    fprintf(fd, "%lf,%lf,%d,%d,%d,%u,%lf,%lf,%lf,%d,%d,%d,%lf,%lf\n", get_time_ms(), deltaT, data.battery_voltage, data.HL_cpu_load, data.angle_yaw,
            ctrl.u_thrust, Quadptr->I_x, Quadptr->I_y, Quadptr->I_z, /*ctrl.roll_d, ctrl.pitch_d,*/ ctrl.yaw_d, Quadptr->roll, Quadptr->pitch);

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
    double states[12];
    kalmanFilter(states, Quad->I_x, Quad->I_y, Quad->I_z);

    // TODO: test before use
    Quad->I_x_kal = states[0];
    Quad->I_x_dot_kal = states[1];
    Quad->I_x_ddot_kal = states[2];
    Quad->I_x_dddot_kal = states[3];
    Quad->I_y_kal = states[4];
    Quad->I_y_dot_kal = states[5];
    Quad->I_y_ddot_kal = states[6];
    Quad->I_y_dddot_kal = states[7];
    Quad->I_z_kal = states[8];
    Quad->I_z_dot_kal = states[9];
    Quad->I_z_ddot_kal = states[10];
    Quad->I_z_dddot_kal = states[11];

    Quad->Q_roll_kal = states[12];
    Quad->Q_roll_dot_kal = states[13];
    Quad->Q_roll_ddot_kal = states[14];
    Quad->Q_roll_dddot_kal = states[15];

    Quad->Q_pitch_kal = states[16];
    Quad->Q_pitch_dot_kal = states[17];
    Quad->Q_pitch_ddot_kal = states[18];
    Quad->Q_pitch_dddot_kal = states[19];

    Quad->Q_yaw_kal = states[20];
    Quad->Q_yaw_dot_kal = states[21];
    Quad->Q_yaw_ddot_kal = states[22];
    Quad->Q_yaw_dddot_kal = states[23];

    // END critical section
    return 0;
}

void plotPath(double t1) {
    FILE* gnuplot = popen("gnuplot -persist", "w");  // -persist
    if (gnuplot == NULL) {
        return NULL;
    }
    fprintf(gnuplot, "set grid\n set  mxtics 4\n set mytics 4\n");
    fprintf(gnuplot, "plot '-' using 1:2 with lines title 'a' lw 2,'-' using 1:2 with lines title 'v' lw 2,'-' using 1:2 with lines title 's' lw 2 \n");  // linespoints
    double output[6] = {};
    double t, t0, a_max, v_max, s_d;
    t = 0;
    t0 = 2;        // s
    a_max = 2;     // m/s^2
    v_max = 0.05;  // m/s
    s_d = 0.5;     // m

    continousPath(output, t, t0, a_max, v_max, s_d);
    printf("duration: %lf\n", output[3]);
    for (int k = 0; k < 3; k++) {
        for (double i = 0; i < output[3]; i = i + 0.05) {
            continousPath(output, i, t0, a_max, v_max, s_d);
            // printf("%d,%lf\n", k, output[k]);
            fprintf(gnuplot, "%lf %lf\n", i, output[k]);
        }
        fprintf(gnuplot, "e\n");
    }

    fclose(gnuplot);
}