#include "../include/missionPlanning.h"

#include <math.h>
#include <stdio.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/MPC.h"
#include "../include/pid.h"
#include "../include/quad.h"

/*
// MPC based controller
int pathMPC(double Q_safeZ, double Q_safeX, double Q_safeY, double I_safeZ, double I_safeX, double I_safeY, struct QuadState* Quad, struct CONTROL* ctrl, double actTime, double Ft_i[4]) {
    // feed forward to overcome gravity --> acquired from measurement data thrust0 = 104
    ctrl->u_thrust = 104 * 0;

    // prepare MPC state variables

    // double x0[12] = {(Q_safeX - Quad->Q_x_kal) / 1000.0, (Q_safeY - Quad->Q_y_kal) / 1000.0, (Q_safeZ - Quad->Q_z_kal) / 1000.0, (-Quad->Q_roll_kal), (-Quad->Q_pitch_kal), (-Quad->Q_yaw_kal),
    //           (-Quad->Q_x_dot_kal) / 1000.0, (-Quad->Q_y_dot_kal) / 1000.0, (-Quad->Q_z_dot_kal) / 1000.0, (-Quad->Q_roll_dot_kal), (-Quad->Q_pitch_dot_kal), (-Quad->Q_yaw_dot_kal)};
    // double x0[12] = {(I_safeX-Quad->I_x)/5000.0,(I_safeY-Quad->I_y)/5000.0,0, (-Quad->Q_roll_kal),(-Quad->Q_pitch_kal),Quad->Q_yaw_kal/10.0,
    //                 0,0,0,(Quad->Q_roll_dot_kal)/20.0, (Quad->Q_pitch_dot_kal)/20.0, -(Quad->Q_yaw_dot_kal)/20.0};
    // double x0[12] = {0*(Q_safeX-Quad->Q_x_kal)/5000.0,0*(Q_safeY-Quad->Q_y_kal)/5000.0,0, (-Quad->Q_roll_kal),(-Quad->Q_pitch_kal),Quad->Q_yaw_kal/10.0,
    //                 0,0,0,(Quad->Q_roll_dot_kal)/20.0, (Quad->Q_pitch_dot_kal)/20.0, -(Quad->Q_yaw_dot_kal)/20.0};
    // double x0[12] = {0*(Q_safeX-Quad->Q_x_kal)/5000.0,0*(Q_safeY-Quad->Q_y_kal)/5000.0,0, (-Quad->Q_roll_kal),(-Quad->Q_pitch_kal),Quad->Q_yaw_kal/10.0,
    // 0,0,0,0,0,0};

    // double x0[12] = {(I_safeX - Quad->I_x_kal) / 1000.0, (I_safeY - Quad->I_y_kal) / 1000.0, (I_safeZ - Quad->I_z_kal) / 1000.0, (-Quad->I_roll_kal), (-Quad->I_pitch_kal), (-Quad->I_yaw_kal),
    //           (-Quad->I_x_dot_kal) / 1000.0, (-Quad->I_y_dot_kal) / 1000.0, (-Quad->I_z_dot_kal) / 1000.0, (-Quad->I_roll_dot_kal), (-Quad->I_pitch_dot_kal), (-Quad->I_yaw_dot_kal)};

    // double x0[12] = {0, 0, 0, (-Quad->I_roll_kal), (-Quad->I_pitch_kal), (-Quad->I_yaw_kal),
    //                  0, 0, 0, (-Quad->I_roll_dot_kal), (-Quad->I_pitch_dot_kal), (-Quad->I_yaw_dot_kal)};

    double x0[12] = {0, 0, 0, (-Quad->I_roll_kal), (-Quad->I_pitch_kal), (-0 * Quad->I_yaw_kal),
                     0, 0, 0, (-Quad->I_roll_dot_kal) / 400.0, (-Quad->I_pitch_dot_kal) / 400.0, (-0 * Quad->I_yaw_dot_kal) / 400.0};

    // solve unconstrained optimal control problem
    solveOCP(Ft_i, x0);

    // printf("\n%lf,%lf,%lf,%lf\n", Ft_i[0], Ft_i[1], Ft_i[2], Ft_i[3]);

    // convert force to u
    double C_T = 8.5041e-4;
    double power = 7.0 / 4.0;

    // assign ctrl to be sent to the quadrocopter
    double F_equi = (1.3068 - 0.45) * 9.81 / 4.0;  // (m_body + 4 * m_rotor)*g / 4
    for (int i = 0; i < 4; i++) {
        Ft_i[i] *= 0.80;
        if (Ft_i[i] >= 0) {
            ctrl->u_i[i] = (int)pow((Ft_i[i]) / C_T, 1 / power);
        } else {
            ctrl->u_i[i] = (int)-pow((-Ft_i[i]) / C_T, 1 / power);
        }
    }
}
*/

//PID Controller for a hovering flight with onboard drone controller
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
    // continousPath(output, actTime, t0, a_max, v_max, s_d);

    // kalman
    updatePID_statespace(pidx, actTime, (I_safeX - Quad->I_x_kal) / 1000.0, -Quad->I_x_dot_kal / 1000.0);
    updatePID_statespace(pidy, actTime, (I_safeY - Quad->I_y_kal) / 1000.0, -Quad->I_y_dot_kal / 1000.0);
    updatePID_statespace(pidz, actTime, (height - Quad->I_z_kal) / 1000.0, -Quad->I_z_dot_kal / 1000.0);

    // asin() is only defined for range [-1,1] therefore limiting is needed
    x_ddot = pidx->currentValue < 1 ? pidx->currentValue : 1;
    x_ddot = x_ddot > -1 ? x_ddot : -1;

    y_ddot = pidy->currentValue < 1 ? pidy->currentValue : 1;
    y_ddot = y_ddot > -1 ? y_ddot : -1;

    // feed forward to overcome gravity --> acquired from measurement data thrust0 = 104
    unsigned char thrust0 = 101;

    // assign pid to u_thrust only if positive
    double thrust = (pidz->currentValue + thrust0) >= 0 ? pidz->currentValue + thrust0 : 1;
    ctrl->u_thrust = thrust < 200 ? thrust : 200;
    // ctrl->u_thrust = thrust0;
    // printf("u_thrust,%u,", ctrl->u_thrust);

    double q6 = Quad->yaw;

    // calculate angles in order to move quadrocopter in KI_(xy)-plane

    // TODO: how does this work (where does the equation come from)
    double roll_d = -asin((y_ddot * cos(q6) + x_ddot * sin(q6)) / g);
    double pitch_d = -asin((x_ddot * cos(q6) - y_ddot * sin(q6)) / cos(roll_d) / g);

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
    int maxDelta = 5;  // 5 degrees max
    ctrl->yaw_d = (210 - 45);

    if (fabs(ctrl->roll_d) > 32760 || fabs(ctrl->pitch_d) > 32760) {
        ctrl->roll_d = 0;
        ctrl->pitch_d = 0;
    }

    // assign yaw command
    int yaw_delta = ctrl->yaw_d - data.angle_yaw;

    if (yaw_delta > maxDelta) {
        ctrl->yaw_d = data.angle_yaw + maxDelta;
    } else if (yaw_delta < -maxDelta) {
        ctrl->yaw_d = data.angle_yaw - maxDelta;
    }

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
