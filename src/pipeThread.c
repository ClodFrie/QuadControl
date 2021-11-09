#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../include/quad.h"
#include "../include/threads.h"

void* pipeThread(void* vptr) {
    // declare this as a "real-time" task
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 10;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        printf("sched_setscheduler failed\n");
        exit(-1);
    }

    // cast pointer to QuadState type
    struct QuadState* Quadptr = (struct QuadState*)vptr;

    FILE* fp = popen("/usr/bin/python3 \"Calculate_Points+RealTime.py\"", "r");
    if (fp == NULL) {
        printf("popen error\n");
    }
    usleep(100000);

    printf("[Qualisys] ready to read\n");
    char inLine[1024];
    while (fgets(inLine, sizeof(inLine), fp) != NULL) {
        // printf("%s",inLine); //DEBUG
        if (pthread_mutex_trylock(&state_mutex) == 0) {  // lock succesful
            sscanf(inLine, "[ %lf %lf %lf],[ %lf %lf %lf]",
                   &(Quadptr->Q_x), &(Quadptr->Q_y), &(Quadptr->Q_z), &(Quadptr->roll), &(Quadptr->pitch), &(Quadptr->yaw));  // read data
            pthread_mutex_unlock(&state_mutex);                                                                               // unlock mutex
        }
    }

    printf("close pipe\n");
    pclose(fp);

    return NULL;
}