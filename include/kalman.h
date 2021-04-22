/* kalman.h */
// idea from https://stackoverflow.com/questions/26889142/using-eigen-in-a-c-project

#ifndef KALMAN_H
#define KALMAN_H

    #ifdef __cplusplus
    extern "C" {
    #endif /* __cplusplus */

    int kalmanFilter(double state[],double ix, double iy,double iz);
    int initKalman();

    #ifdef __cplusplus
    } /* extern "C" */
    #endif /* __cplusplus */

#endif
