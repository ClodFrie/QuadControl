#include <stdio.h>
#include <unistd.h>

#include "../include/quad.h"
#include "../include/threads.h"

void* pipeThread(void* vptr) {
    FILE* fp = popen("python3 \"Calculate_Points+RealTime.py\"", "r");
    if (fp == NULL) {
        printf("popen error\n");
    }
    usleep(100000);

    struct QuadState* Quad = (struct QuadState*)vptr;



    printf("ready to read\n");
    char inLine[1024];
    while (fgets(inLine, sizeof(inLine), fp) != NULL) {
        if (pthread_mutex_trylock(&state_mutex) == 0) {  // lock succesful
            sscanf(inLine, "[ %lf %lf %lf],[ %lf %lf %lf]",
                   &(Quad->Qx), &(Quad->Qy), &(Quad->Qz), &(Quad->roll), &(Quad->pitch), &(Quad->yaw));  // read data
            pthread_mutex_unlock(&state_mutex);                                                          // unlock mutex
        }
    }

    printf("close pipe\n");
    pclose(fp);

    return NULL;
}