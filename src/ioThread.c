#include <ftd2xx.h>
#include <math.h>
#include <sched.h>  // rt patch
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>  // for bzero
#include <unistd.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/FTDI_helpers.h"
#include "../include/MPC.h"
#include "../include/crc.h"
#include "../include/kalman.h"
#include "../include/missionPlanning.h"
#include "../include/pid.h"
#include "../include/quad.h"
#include "../include/serial_helpers.h"
#include "../include/threads.h"
#include "../include/time_helper.h"

// function prototypes
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
    IMUC_INIT,
    IMUC_HOVER,
    IMUC_ONLINE_TRAJECTORY,
    LANDING
} state;
// IMUC = IMU + Camera

PID pidx, pidy, pidz;  // create pid variables

void* ioThread(void* vptr) {
    // declare this as a "real-time" task
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 10;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        printf("sched_setscheduler failed\n");
        exit(-1);
    }

    // cast pointer to QuadState type
    struct QuadState* Quadptr = (struct QuadState*)vptr;

    // define starting state as INIT
    state = INIT;
    // state = IMUC_INIT;

    int fdSerial;
    FT_HANDLE ftHandle;

    // openFTDI(&ftHandle);
    fdSerial = openPort('0' - '0');  // TODO: 'F'
    if (fdSerial == -1) {
        return NULL;
    }

    // zero all variables
    bzero(&data, sizeof(data));
    bzero(&ctrl, sizeof(ctrl));
    bzero(&params, sizeof(params));

    // initialize kalman filter
    // TODO: get kalman filter going for ultrasonic measurements
    initKalman();
    printf("Kalman started\n");

    // initialize MPC
    initMPC();

    double t1 = get_time_ms();
    // write control parameters
    setParams(0.007265, 0.008265 + 0.004, 0.006500, 0.0011250, 0, 0);

    while (/*sendParams(&ftHandle)*/ sendParameters(fdSerial) != 0) {
        ;  // make sure that parameters have been received
    }
    printf("[PARAM] received succesfully!\n");

    fflush(0);

    // plotPath(t1);  // gnuplot

    double I_safeX0 = 0, I_safeY0 = 0;
    unsigned long cnt = 0;

    while (1) {  // do forever

        if (pthread_mutex_lock(&state_mutex) == 0) {
            updateState(Quadptr);
            Quadptr->quadTime = get_time_ms();  // update time
            // release mutex
            pthread_mutex_unlock(&state_mutex);

            // receive drone data
            if (cnt % 5 == 0 || data.battery_voltage == 0) {  // every n-th time or when data was not received properly
                requestData_ser(fdSerial);
                // requestData(&ftHandle);
            }
            cnt++;

            // calculate desired movements
            printf("st: %d,", state);

            switch (state) {
                case INIT:  // wait for measurements from Qualisys system

                    // TODO: send 0 to all motors, angles, etc.

                    if (Quadptr->I_z >= 200) {
                        // additional criteria, wait for kalman filter to converge
                        double delta = 0.01;  // 0.01 mm deviation
                        if (fabs(Quadptr->I_x - Quadptr->I_x_kal) < delta && fabs(Quadptr->I_y - Quadptr->I_y_kal) < delta) {
                            Quadptr->trajectory.I_x = Quadptr->I_x;
                            Quadptr->trajectory.I_y = Quadptr->I_y;
                            Quadptr->trajectory.I_z = Quadptr->I_z;
                            I_safeX0 = Quadptr->I_x;
                            I_safeY0 = Quadptr->I_y;

                            initPID(&pidx, 3.8, 0.9, 3.0, 8.20, get_time_ms());  // pidX
                            initPID(&pidy, 3.8, 2.2, 3.0, 6.25, get_time_ms());  // pidY

                            initPID(&pidz, 3.2, 1.5, 1.5, 20, get_time_ms());  // pidZ

                            state = IDLE;  // change active state to idle
                        }
                    }
                    break;
                case IDLE:  // wait for some kind of start signal
                    // send zero throttle
                    bzero(&ctrl, sizeof(ctrl));
                    ctrl.u_thrust = 0;
                    ctrl.pitch_d = 0;
                    ctrl.roll_d = 0;
                    ctrl.yaw_d = data.angle_yaw;

                    // flip the analog switch "start/land"
                    if (data.channel[2] > 10) {
                        state = LEMNISCATE_TRAJECTORY;
                        // state = TAKEOFF;
                        Quadptr->quadStartTime = get_time_ms();
                    }

                    break;
                case TAKEOFF:

                    // currently there is no takeoff state
                    // state = HOVER;

                    // pathMPC(I_safeX0, I_safeY0, Quadptr, &ctrl, get_time_ms());
                    // pathPID(I_safeZ,I_safeX,I_safeY, Quadptr, &ctrl, get_time_ms(), Ft_i);
                    calculateHover(I_safeX0, I_safeY0, 7 /*degrees*/, Quadptr, &ctrl, &pidx, &pidy, &pidz);
                    break;
                case HOVER:

                    break;
                case RECTANGULAR_TRAJECTORY:
                    break;
                case LEMNISCATE_TRAJECTORY:
                    lemniscateHover(I_safeX0, I_safeY0, 7 /*degrees*/, Quadptr, &ctrl, &pidx, &pidy, &pidz);
                    break;
                case IMUC_INIT:
                    ctrl.u_thrust = 0;
                    ctrl.pitch_d = 0;
                    ctrl.roll_d = 0;
                    ctrl.yaw_d = data.angle_yaw;

                    if (data.channel[2] > 10) {
                        initPID(&pidx, 2.5, 0.45, 0.05, 1.00, get_time_ms());  // pidX
                        initPID(&pidy, 2.5, 0.75, 0.05, 1.00, get_time_ms());  // pidY

                        initPID(&pidz, 0.1, 0, 0, 10, get_time_ms());  // pidZ
                        state = IMUC_HOVER;
                    }
                    break;

                case IMUC_HOVER:
                    calculateIMUCHover(get_time_ms(), 0.3, Quadptr, 5, &pidx, &pidy, &pidz);
                    break;
                case IMUC_ONLINE_TRAJECTORY:
                    break;
                default:
                    printf("illegal state\n");
                    break;
            }

            // write control commands - only if new data has been generated
            sendCommand(fdSerial);
            // sendCmd(&ftHandle);

            Quadptr->newDataAvailable = 1;  //TODO: set data_available flag for logging // TODO find correct place for this
        }
    }

    // free resources
    if (fdSerial != -1) {
        close(fdSerial);
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

    params.safeAngle = 15;  // degrees will be later multiplied on quadrotor

    params.CRC = crc8(0, (unsigned char*)(&params), sizeof(params) - 1);
}

int updateState(struct QuadState* Quad) {
    // START critical section
    double q4 = Quad->roll;
    double q5 = Quad->pitch;
    double q6 = Quad->yaw;

    double Qx0 = Quad->Q_x;
    double Qy0 = Quad->Q_y;
    double Qz0 = Quad->Q_z;

    // state estimation: kalman filter
    double states[24];
    kalmanFilter(states, Quad->I_x, Quad->I_y, Quad->I_z, q4, q5, q6);

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

    Quad->I_roll_kal = states[12];
    Quad->I_roll_dot_kal = states[13];
    Quad->I_roll_ddot_kal = states[14];
    Quad->I_roll_dddot_kal = states[15];

    Quad->I_pitch_kal = states[16];
    Quad->I_pitch_dot_kal = states[17];
    Quad->I_pitch_ddot_kal = states[18];
    Quad->I_pitch_dddot_kal = states[19];

    Quad->I_yaw_kal = states[20];
    Quad->I_yaw_dot_kal = states[21];
    Quad->I_yaw_ddot_kal = states[22];
    Quad->I_yaw_dddot_kal = states[23];

    // Quad->system_speeds[0] = cos(q5) * cos(q6) * Quad->I_x_dot_kal - cos(q5) * sin(q6) * Quad->I_y_dot_kal + sin(q5) * Quad->I_z_dot_kal;
    // Quad->system_speeds[1] = (-sin(q5) * sin(q6) * Quad->I_y_dot_kal + cos(q6) * sin(q5) * Quad->I_x_dot_kal - Quad->I_z_dot_kal * cos(q5)) * sin(q4) - cos(q4) * (sin(q6) * Quad->I_x_dot_kal + cos(q6) * Quad->I_y_dot_kal);
    // Quad->system_speeds[2] = (-sin(q5) * sin(q6) * Quad->I_y_dot_kal + cos(q6) * sin(q5) * Quad->I_x_dot_kal - Quad->I_z_dot_kal * cos(q5)) * cos(q4) + sin(q4) * (sin(q6) * Quad->I_x_dot_kal + cos(q6) * Quad->I_y_dot_kal);
    // Quad->system_speeds[3] = Quad->Q_roll_dot_kal - sin(q5) * Quad->Q_yaw_dot_kal;
    // Quad->system_speeds[4] = cos(q4) * Quad->Q_pitch_dot_kal + sin(q4) * cos(q5) * Quad->Q_yaw_dot_kal;
    // Quad->system_speeds[5] = -sin(q4) * Quad->Q_pitch_dot_kal + cos(q4) * cos(q5) * Quad->Q_yaw_dot_kal;

    Quad->I_x = cos(q5) * cos(q6) * Qx0 + (sin(q4) * sin(q5) * cos(q6) - cos(q4) * sin(q6)) * Qy0 + (cos(q4) * sin(q5) * cos(q6) + sin(q4) * sin(q6)) * Qz0;
    Quad->I_y = -(cos(q5) * sin(q6) * Qx0 + (sin(q4) * sin(q5) * sin(q6) + cos(q4) * cos(q6)) * Qy0 + (cos(q4) * sin(q5) * sin(q6) - sin(q4) * cos(q6)) * Qz0);
    Quad->I_z = -(-sin(q5) * Qx0 + sin(q4) * cos(q5) * Qy0 + cos(q4) * cos(q5) * Qz0);

    // END critical section
    return 0;
}

void plotPath(double t1) {
    FILE* gnuplot = popen("gnuplot -persist", "w");  // -persist
    if (gnuplot == NULL) {
        return;
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
