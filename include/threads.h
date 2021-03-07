#ifndef THREADS_H
#define THREADS_H

#include <pthread.h>

void* ioThread(void*);
void* consoleThread(void*);
void* pipeThread(void*);

extern pthread_mutex_t state_mutex;
#endif