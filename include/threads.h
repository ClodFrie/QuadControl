#ifndef THREADS_H
#define THREADS_H

#include <pthread.h>

void* ioThread(void*);
void* serialThread(void*);
void* pipeThread(void*);
void* cameraThread(void*);
void* logThread(void*);

extern pthread_mutex_t state_mutex;
#endif