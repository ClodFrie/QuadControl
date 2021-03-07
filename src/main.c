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
    setvbuf(stdout,NULL,_IONBF,0);

    struct QuadState Quad;

    pthread_t p_ioThread, p_consoleThread, p_pipeThread;

    pthread_create(&p_ioThread, NULL, ioThread, &Quad);
    pthread_create(&p_consoleThread, NULL, consoleThread, NULL);
    pthread_create(&p_pipeThread, NULL, pipeThread, &Quad);

    pthread_join(p_ioThread, NULL);
    pthread_join(p_consoleThread, NULL);
    pthread_join(p_pipeThread, NULL);  // TODO: After getting controller right, send position commands based on localization...
}

// open gnuplot
// gnuplot = popen("gnuplot -persist", "w");
// if (gnuplot == NULL) {
//     return 0;
// }