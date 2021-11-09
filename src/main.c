#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>

#include "../../../asctec-sdk3.0-archive/sdk.h"         // TEMPORARY: all data available to sdk.c
#include "../../../asctec-sdk3.0-archive/serialcomm.h"  // data types for serial communication
#include "../include/quad.h"
#include "../include/threads.h"

// global variables
pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER;

struct DATA data;
struct CONTROL ctrl;
struct PARAMS params;
struct QuadState Quad;

int main() {
    // intro message
    printf("Hello QuadControl\n");

    // make printfs immediate (no buffer)
    setvbuf(stdout, NULL, _IONBF, 0);

    struct QuadState Quad;

    pthread_t p_ioThread, p_logThread, p_serialThread, p_pipeThread, p_cameraThread;

    pthread_create(&p_ioThread, NULL, ioThread, &Quad);
    pthread_create(&p_logThread, NULL, logThread, &Quad);
    pthread_create(&p_pipeThread, NULL, pipeThread, &Quad); // Qualisys Python Pipe
    // pthread_create(&p_serialThread, NULL, serialThread, &Quad);  // Ultrasonic Sensor Pipe
    // pthread_create(&p_cameraThread, NULL, cameraThread, &Quad); // Camera Evaluation Pipe

    pthread_join(p_ioThread, NULL);
    pthread_join(p_logThread, NULL);
    pthread_join(p_pipeThread, NULL);// Qualisys Python Pipe
    // pthread_join(p_serialThread, NULL);  // Ultrasonic Sensor Pipe
    // pthread_join(p_cameraThread, NULL);// Camera Evaluation Pipe

    return 0;
}

// open gnuplot
// gnuplot = popen("gnuplot -persist", "w");
// if (gnuplot == NULL) {
//     return 0;
// }