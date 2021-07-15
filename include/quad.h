#ifndef QUAD_H
#define QUAD_H

struct QuadState {
    // Quadrotor Frame
    double Q_x;
    double Q_y;
    double Q_z;

    double roll;
    double pitch;
    double yaw;

    // Inertial Frame
    double I_x, I_y, I_z;

    // Inertial Frame filtered
    double I_x_kal, I_y_kal, I_z_kal;
    double I_x_dot_kal, I_x_ddot_kal, I_x_dddot_kal, I_y_dot_kal, I_y_ddot_kal, I_y_dddot_kal, I_z_dot_kal, I_z_ddot_kal, I_z_dddot_kal;
    
    double I_roll_kal,I_pitch_kal,I_yaw_kal;
    double I_roll_dot_kal,I_pitch_dot_kal,I_yaw_dot_kal,I_roll_ddot_kal,I_pitch_ddot_kal,I_yaw_ddot_kal,I_roll_dddot_kal,I_pitch_dddot_kal,I_yaw_dddot_kal;

    // time for trajectory planning
    double quadStartTime;
    double trajectoryStartTime;

    // Body Frame filtered
    // double Q_x_kal, Q_y_kal, Q_z_kal;
    // double Q_x_dot_kal, Q_x_ddot_kal, Q_x_dddot_kal, Q_y_dot_kal, Q_y_ddot_kal, Q_y_dddot_kal, Q_z_dot_kal, Q_z_ddot_kal, Q_z_dddot_kal;

    // double Q_roll_kal,Q_pitch_kal,Q_yaw_kal;
    // double Q_roll_dot_kal,Q_pitch_dot_kal,Q_yaw_dot_kal,Q_roll_ddot_kal,Q_pitch_ddot_kal,Q_yaw_ddot_kal,Q_roll_dddot_kal,Q_pitch_dddot_kal,Q_yaw_dddot_kal;

    // double system_speeds[6];
};

#endif