#ifndef QUAD_H
#define QUAD_H

struct QuadState {
    // Quadrotor Frame
    double Qx;
    double Qy;
    double Qz;

    double roll;
    double pitch;
    double yaw;

    // Inertial Frame
    double I_x, I_y, I_z;

    // Inertial Frame filtered
    double I_x_kal, I_y_kal, I_z_kal;
    double I_x_dot_kal, I_y_dot_kal, I_z_dot_kal;
};

#endif