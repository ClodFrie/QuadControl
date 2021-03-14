#ifndef QUAD_H
#define QUAD_H

struct QuadState{
    // Quadrotor Frame
    double Qx;
    double Qy;
    double Qz;

    double roll;
    double pitch;
    double yaw;


    // Inertial Frame
    double I_x,I_y,I_z;
 
};


#endif