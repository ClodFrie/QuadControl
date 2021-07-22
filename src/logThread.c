#include <math.h>
#include <quad.h>
#include <stdio.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"  // data types for serial communication
#include "../include/threads.h"
#include "../include/time_helper.h"

FILE* initLogFile(char* mType);
int writeLogLine(FILE* fd, double deltaT, double I_safeX, double I_safeY, double I_safeZ, struct QuadState* Quadptr, double Ft_i[]);

void* logThread(void* vptr) {
    // cast pointer to QuadState type
    struct QuadState* Quadptr = (struct QuadState*)vptr;

    // initialize log file
    FILE* fd = initLogFile("IDLE.X");

    while (1) {
        // write logfile
        if (Quadptr->newDataAvailable == 1) {
            if (pthread_mutex_trylock(&state_mutex)) {
                writeLogLine(fd, get_time_ms() - Quadptr->quadTime, Quadptr->trajectory.I_x, Quadptr->trajectory.I_y, Quadptr->trajectory.I_z, Quadptr, NULL);
                Quadptr->newDataAvailable = 0;
                pthread_mutex_unlock(&state_mutex);
            }
        }
    }
    fclose(fd);
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
    fprintf(fd, "T,dT,BAT,CPU,YAW_QUAD,U_THRUST,I_safeX,I_safeY,I_safeZ,I_X,I_Y,I_Z,B_z,B_dist,B_angle,ROLL,PITCH,YAW,ROLL_dot,PITCH_dot,YAW_dot,Ft1,Ft2,Ft3,Ft4,Camera_Distance\n");

    // return file descriptor for use in other functions
    return fd;
}

// write current data for every time step
int writeLogLine(FILE* fd, double deltaT, double I_safeX, double I_safeY, double I_safeZ, struct QuadState* Quadptr, double Ft_i[]) {
    fprintf(fd, "%lf,%lf,%d,%d,%d,%u,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d\n", get_time_ms(), deltaT, data.battery_voltage, data.HL_cpu_load, data.angle_yaw,
            ctrl.u_thrust, I_safeX, I_safeY, I_safeZ, Quadptr->I_x, Quadptr->I_y, Quadptr->I_z, Quadptr->IMUC.B_distance0, Quadptr->IMUC.B_averageDistance, Quadptr->IMUC.angle_yaw, Quadptr->I_roll_kal,
            Quadptr->I_pitch_kal, Quadptr->I_yaw_kal, Quadptr->I_roll_dot_kal, Quadptr->I_pitch_dot_kal, Quadptr->I_yaw_dot_kal, /*Ft_i[0], Ft_i[1], Ft_i[2], Ft_i[3],*/0,0,0,0, Quadptr->IMUC.C_distance);

    // fflush(fd);  // flush file buffer, so that every line is written and not lost when aborting execution [CTRL]+[C]

    // print debug messages
    printf("[T],%3.2lf,", deltaT);
    printf("BAT,%5d,CPU,%3d,yaw,%3d,", data.battery_voltage, data.HL_cpu_load, data.angle_yaw);

    printf("I_x,%6.2f,I_y,%6.2f,I_z,%6.2f,B_z,%6.2lf,B_dist,%6.2lf,B_angle,%2.2lf,roll,%2.1lf,pitch,%2.1lf,yaw,%2.1lf,roll_cmd:%3d,pitch_cmd:%3d,yaw_cmd:%d\n", Quadptr->I_x, Quadptr->I_y, Quadptr->I_z, Quadptr->IMUC.B_distance0, Quadptr->IMUC.B_averageDistance, Quadptr->IMUC.angle_yaw, Quadptr->I_roll_kal * 180.0 / M_PI, Quadptr->I_pitch_kal * 180.0 / M_PI, Quadptr->I_yaw_kal * 180.0 / M_PI, ctrl.roll_d / 1000, ctrl.pitch_d / 1000, ctrl.yaw_d);
    // printf("roll%lf,pitch%lf,yaw%d",data.angle_roll/1000.0,data.angle_pitch/1000.0,data.angle_yaw);
    // printf("u:%d,%d,%d,%d\n",data.u[0],data.u[1],data.u[2],data.u[3]);
}
