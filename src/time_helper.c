#include "../include/time_helper.h"

#include <stdlib.h>
#include <sys/time.h>

double get_time_ms() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec + (t.tv_usec / 1000000.0)) * 1000.0;
}
