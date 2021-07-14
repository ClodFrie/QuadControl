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
    int fd = openPort(0);
    if (fd < 0) {
        printf("[SERIAL] open port failed");
        exit(1);
    }
    usleep(100000);

    printf("[SERIAL] ready to read\n");
    char inLine[1024];
    while (1 /*fgets(inLine, sizeof(inLine), fd) != NULL*/) {
        // printf("%s",inLine); //DEBUG
        if (pthread_mutex_trylock(&state_mutex) == 0) {  // lock succesful
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

            double time, dist0, dist1, dist2, dist3, angle;
            sscanf(inLine, "%lfs\t%lf\t%lf\t%lf\t%lf\t%lf\n", &time, &dist0, &dist1, &dist2, &dist3, &angle);

            pthread_mutex_unlock(&state_mutex);  // unlock mutex
        }
    }

    printf("close pipe\n");
    close(fd);

    return NULL;
}