/* kalman.h */
// idea from https://stackoverflow.com/questions/26889142/using-eigen-in-a-c-project

#ifndef KALMAN_H
#define KALMAN_H

    #ifdef __cplusplus
    extern "C" {
    #endif /* __cplusplus */

    int solveOCP(double Ft_i[],double state[]);
    int initMPC();

    #ifdef __cplusplus
    } /* extern "C" */
    #endif /* __cplusplus */

#endif
