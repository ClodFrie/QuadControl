#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../include/quad.h"
#include "../include/threads.h"

void* cameraThread(void* vptr) {
    // declare this as a "real-time" task
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO) - 10;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        printf("sched_setscheduler failed\n");
        exit(-1);
    }

    // cast pointer to QuadState type
    struct QuadState* Quadptr = (struct QuadState*)vptr;

    FILE* fp = popen("/usr/bin/python3 \"../Video/VideoProcessing_headless.py\"", "r");
    if (fp == NULL) {
        printf("popen error\n");
    }
    // TODO: convert camera coordinates to Wall coordinates
    usleep(10000);

    printf("[Camera] ready to read\n");
    char inLine[1024];
    while (fgets(inLine, sizeof(inLine), fp) != NULL) {
        // printf("%s", inLine);                            //DEBUG
            double fps;
            int C_distance;
            sscanf(inLine, "[%lffps,%dpx]\n", &fps, &C_distance);  // read data
            // printf("\nC_distance:%d\n", /*Quadptr->IMUC.*/C_distance);
        if (pthread_mutex_trylock(&state_mutex) == 0) {  // lock succesful
            Quadptr->IMUC.C_distance = C_distance;
            pthread_mutex_unlock(&state_mutex);  // unlock mutex
        }
    }

    printf("close pipe\n");
    pclose(fp);

    return NULL;
}