#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/quad.h"
#include "../include/serial_helpers.h"
#include "../include/threads.h"
#include "../include/time_helper.h"

void* serialThread(void* vptr) {
    // declare this as a "real-time" task
    // struct sched_param param;
    // param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 10;
    // if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    //     printf("sched_setscheduler failed\n");
    //     exit(-1);
    // }

    // cast pointer to QuadState type
    struct QuadState* Quadptr = (struct QuadState*)vptr;

    // open serial port
    int fd = openPort('0'-'0');
    if (fd < 0) {
        printf("[SERIAL] open port failed. Closing Application!\n");
        exit(1);
    }
    usleep(10000);

    printf("[SERIAL] ready to read\n");
    
    char inLine[1024];
    while (1 /*fgets(inLine, sizeof(inLine), fd) != NULL*/) {
        // printf("%s",inLine); //DEBUG
        char endofline = 0;
        int tmpRec = 0;
        double t0 = get_time_ms();
        while (!endofline) {
            tmpRec += read(fd, inLine + tmpRec, 20);  // read serial data
            for (int i = 0; i < tmpRec; i++) {        //synchronize to '\n'
                if (inLine[i] == '\n') {
                    endofline = 1;
                    inLine[i + 1] = '\0';  // end string
                    break;
                }
            }
        }


        if (pthread_mutex_trylock(&state_mutex) == 0) {  // lock succesful
            double time, dist0, dist1, dist2, dist3, angle;
            sscanf(inLine, "%lfs\t%lf\t%lf\t%lf\t%lf\t%lf\n", &time, &Quadptr->IMUC.B_distance0,&Quadptr->IMUC.B_distance1, &Quadptr->IMUC.B_distance2, &Quadptr->IMUC.B_averageDistance, &Quadptr->IMUC.angle_yaw);
            pthread_mutex_unlock(&state_mutex);  // unlock mutex
            // printf(inLine);
        }
    }

    printf("close pipe\n");
    close(fd);

    return NULL;
}