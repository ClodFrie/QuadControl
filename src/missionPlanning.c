#include "../include/missionPlanning.h"

#include <math.h>
#include <stdio.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/MPC.h"
#include "../include/pid.h"
#include "../include/quad.h"

// MPC based controller
int pathMPC(double height, double I_safeX, double I_safeY, struct QuadState* Quad, struct CONTROL* ctrl, double actTime) {
    // feed forward to overcome gravity --> acquired from measurement data thrust0 = 104
    ctrl->u_thrust = 104;

    // prepare MPC variables
    double Ft_i[4];
    double state[12] = {I_safeX - Quad->I_x_kal, I_safeY - Quad->I_y_kal, height - Quad->I_z_kal, Quad->Q_roll_kal, Quad->Q_pitch_kal, Quad->Q_yaw_kal, Quad->I_x_dot_kal, Quad->I_y_dot_kal, Quad->I_z_dot_kal, Quad->Q_roll_dot_kal, Quad->Q_pitch_dot_kal, Quad->Q_yaw_dot_kal};

    // solve unconstrained optimal control problem
    solveOCP(Ft_i, state);

    // convert force to u
    double C_T = 8.5041e-4;
    double power = 7.0 / 4.0;

    // limit input
    for (int i = 0; i < 4; i++) {
        if (Ft_i[i] < 0.001) {
            Ft_i[i] = 0.001;
        } else if (Ft_i[i] > 9) {
            Ft_i[i] = 9;
        }
    }

    ctrl->u_i[0] = pow(Ft_i[0] / C_T, 1 / power);
    ctrl->u_i[1] = pow(Ft_i[1] / C_T, 1 / power);
    ctrl->u_i[2] = pow(Ft_i[2] / C_T, 1 / power);
    ctrl->u_i[3] = pow(Ft_i[3] / C_T, 1 / power);
}

// PID Controller for a hovering flight
int calculateHover(double height, double I_safeX, double I_safeY, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, PID* pidz, double actTime) {
    double g = 9.81;  // m/s^2
    double m = 1.2;   // "kg"
    double x_ddot, y_ddot;

    double maxAngle = maxAngle_deg * M_PI / 180.0;  // in degree to rad

    // path planning
    double output[6] = {};
    double t, t0, a_max, v_max, s_d;
    t = 0;
    t0 = 10;       // s
    a_max = 2;     // m/s^2
    v_max = 0.05;  // m/s
    s_d = 0.5;     // m
    continousPath(output, actTime, t0, a_max, v_max, s_d);

    // kalman
    updatePID_statespace(pidx, actTime, ((I_safeX - output[2]) - Quad->I_x_kal) / 1000.0, output[1] - Quad->I_x_dot_kal / 1000.0);
    updatePID_statespace(pidy, actTime, (I_safeY - Quad->I_y_kal) / 1000.0, -Quad->I_y_dot_kal / 1000.0);
    updatePID_statespace(pidz, actTime, (height - Quad->I_z_kal) / 1000.0, -Quad->I_z_dot_kal / 1000.0);

    // asin() is only defined for range [-1,1] therefore limiting is needed
    x_ddot = pidx->currentValue < 1 ? pidx->currentValue : 1;
    x_ddot = x_ddot > -1 ? x_ddot : -1;

    y_ddot = pidy->currentValue < 1 ? pidy->currentValue : 1;
    y_ddot = y_ddot > -1 ? y_ddot : -1;

    // feed forward to overcome gravity --> acquired from measurement data thrust0 = 104
    unsigned char thrust0 = 104;

    // assign pid to u_thrust only if positive
    double thrust = (pidz->currentValue + thrust0) >= 0 ? pidz->currentValue + thrust0 : 1;
    ctrl->u_thrust = thrust < 200 ? thrust : 200;
    // ctrl->u_thrust = thrust0;
    printf("u_thrust,%u,", ctrl->u_thrust);

    double q6 = -Quad->yaw;  // TODO: fix this -> q6 needs to be in inertial frame

    // calculate angles in order to move quadrocopter in KI_(xy)-plane

    // TODO: how does this work (where does the equation come from)
    double roll_d = -asin((x_ddot * sin(q6) + y_ddot * cos(q6)) / g);
    double pitch_d = -asin((y_ddot * sin(q6) - x_ddot * cos(q6)) / cos(roll_d) / g);

    roll_d = roll_d > maxAngle ? maxAngle : roll_d;       // limit angles
    roll_d = roll_d < -maxAngle ? -maxAngle : roll_d;     // limit angles
    pitch_d = pitch_d > maxAngle ? maxAngle : pitch_d;    // limit angles
    pitch_d = pitch_d < -maxAngle ? -maxAngle : pitch_d;  // limit angles

    // 1 deg == 1000 cts
    double scaling = 1000.0 / 1.0;

    // lowpass filter angle commands
    // roll_d = roll_d * 0.85 + 0.15 * (ctrl->roll_d/(scaling * 180.0 / M_PI)) ;
    // pitch_d = pitch_d * 0.85 + 0.15 * (ctrl->roll_d/(scaling * 180.0 / M_PI)) ;

    // assign angle commands
    ctrl->roll_d = (short)((roll_d * (scaling * 180.0 / M_PI)));
    ctrl->pitch_d = (short)((pitch_d * (scaling * 180.0 / M_PI)));

    // calculate yaw command
    int maxDelta = 5000;  // 10 degrees max
    ctrl->yaw_d = (210 - 45) * 1000;

    // assign yaw command
    int yaw_delta = ctrl->yaw_d - data.angle_yaw * 1000;

    if (yaw_delta > maxDelta) {
        ctrl->yaw_d = data.angle_yaw * 1000 + maxDelta;
    } else if (yaw_delta < -maxDelta) {
        ctrl->yaw_d = data.angle_yaw * 1000 - maxDelta;
    }

    // print debug message
    printf("I_x,%6.2f,I_y,%6.2f,I_z,%6.2f,I_x_dot,%6.2f,I_y_dot,%6.2f,I_z_dot,%6.2f,roll_cmd,%5d,pitch_cmd,%5d,yaw_cmd,%5d,\n", Quad->I_x, Quad->I_y, Quad->I_z, Quad->I_x_dot_kal, Quad->I_y_dot_kal, Quad->I_z_dot_kal, ctrl->roll_d, ctrl->pitch_d, ctrl->yaw_d);

    return 0;
}

int continousPath(double output[], double t, double t0, double a_max, double v_max, double s_d) {
    double tcons, accel, a, v, s, f1, f2, f3, f4, a1, a2;

    tcons = 0;

    accel = (2.0 * s_d) / (pow(M_PI, 2)); /*for continous acceleration*/

    a = (accel < a_max) ? accel : a_max; /*set a to allowed range*/
    v = 1.0 / 2.0 * a * M_PI;            /*calculate speed*/
    s = 1.0 / 2.0 * pow(M_PI, 2) * a;    /*calculate resulting distance*/

    if (v >= v_max) { /*const speed*/
        v = v_max;
        a = (2.0 * v_max) / M_PI;
        tcons = (s_d - 1.0 / 2.0 * pow(M_PI, 2) * a) / (v);
    } else if (s < s_d) { /*const speed*/
        tcons = (s_d - 1.0 / 2.0 * pow(M_PI, 2) * a) / (v);
    }

    /*definition of areas*/
    f1 = (t >= t0 && t < M_PI + t0) ? 1 : 0;
    f2 = (t >= M_PI + t0 && t <= M_PI + tcons + t0) ? 1 : 0;
    f3 = (t > M_PI + tcons + t0 && t <= 2 * M_PI + tcons + t0) ? 1 : 0;
    f4 = (t >= 2 * M_PI + tcons + t0) ? 1 : 0;

    a1 = a * pow(sin(t - t0), 2);
    a2 = a * pow(sin(t - tcons - t0), 2);

    output[0] = a1 * f1 - a2 * f3; /*a*/

    double v1 = -((s_d - tcons * v) * (cos(t - t0) * sin(t - t0) - (t - t0))) / (pow(M_PI, 2)) * f1;
    double v2 = v * f2;
    double v3 = ((s_d - tcons * v) * (cos(t - t0 - tcons) * sin(t - t0 - tcons) + 2 * M_PI - (t - t0 - tcons))) / (pow(M_PI, 2)) * f3;
    double v4 = 0;

    output[1] = v1 + v2 + v3 + v4; /*v*/

    double s1 = 1.0 / 2.0 * ((s_d - tcons * v) * (pow(cos(t - t0), 2) + pow(t - t0, 2) - 1)) / (pow(M_PI, 2)) * f1;
    double s2 = 1.0 / 4.0 * pow(M_PI, 2) * a * f2 + (v * (t - t0 - M_PI)) * f2;
    double bcons = (tcons > 0) ? 1 : 0;
    double s3 = bcons * (s_d - 1.0 / 2.0 * pow(M_PI, 2) * a) * f3 - 1.0 / 2.0 * ((s_d - tcons * v) * (pow(cos(t - t0 - tcons), 2) + 2 * pow(M_PI, 2) - 4 * M_PI * (t - t0 - tcons) + pow(t - t0 - tcons, 2) - 1)) / (pow(M_PI, 2)) * f3;
    double s4 = s_d * f4;
    output[2] = s1 + s2 + s3 + s4; /*s*/

    output[3] = t0 + tcons + 2 * M_PI; /*T*/

    return 0;
}