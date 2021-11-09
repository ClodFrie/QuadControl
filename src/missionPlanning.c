#include "../include/missionPlanning.h"

#include <math.h>
#include <stdio.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/MPC.h"
#include "../include/pid.h"
#include "../include/quad.h"
#include "../include/time_helper.h"

int lemniscatePathX(double output[], double t, double t0);
int lemniscatePathY(double output[], double t, double t0);

// MPC based controller
int pathMPC(double I_safeX0, double I_safeY0, struct QuadState* Quad, struct CONTROL* ctrl, double actTime) {
    // feed forward to overcome gravity --> acquired from measurement data thrust0 = 104
    ctrl->u_thrust = 104 * 0;

    // prepare MPC state variables
    // double x0[12] = {0, 0, 0, (-Quad->I_roll_kal), (-Quad->I_pitch_kal), (-Quad->I_yaw_kal),
    //                  0, 0, 0, (-Quad->I_roll_dot_kal) / 1.0, (-Quad->I_pitch_dot_kal) / 1.0, (-Quad->I_yaw_dot_kal) / 1.0};

    double x0[12] = {0, 0, 0, (-Quad->I_roll_kal), (-Quad->I_pitch_kal), 0 * (-Quad->I_yaw_kal),
                     0, 0, 0, (-Quad->I_roll_dot_kal) / 7.0, (-Quad->I_pitch_dot_kal) / 7.0, 0 * (-Quad->I_yaw_dot_kal) / 20.0};
    // solve unconstrained optimal control problem
    solveOCP(Quad->forces.F_i, x0);

    // printf("%lf,%lf,%lf,%lf\n", Quad->forces.F_i[0], Quad->forces.F_i[1], Quad->forces.F_i[2], Quad->forces.F_i[3]);

    // convert force to u
    double C_T = 8.5041e-4;
    double power = 7.0 / 4.0;

    // assign ctrl to be sent to the quadrocopter
    double F_equi = (1.3068 - 0.45) * 9.81 / 4.0;  // (m_body + 4 * m_rotor)*g / 4
    for (int i = 0; i < 4; i++) {
        Quad->forces.F_i[i] *= 0.80;
        if (Quad->forces.F_i[i] >= 0) {
            ctrl->u_i[i] = (int)pow((Quad->forces.F_i[i]) / C_T, 1 / power);
        } else {
            ctrl->u_i[i] = (int)-pow((-Quad->forces.F_i[i]) / C_T, 1 / power);
        }
    }
}

// IMUC Flight
int calculateIMUCHover(double actTime, double targetHeight, struct QuadState* Quad, double maxAngle_deg, PID* pidx, PID* pidy, PID* pidz) {
    double g = 9.81;                                // m/s^2
    double maxAngle = maxAngle_deg * M_PI / 180.0;  // in degree to rad

    // rotate from camera frame to body frame ("I-frame" because it is aligned with the wall)
    double i_;

    // update controllers
    updatePID(pidx, actTime, 0 - Quad->IMUC.C_distance / 50.0);
    updatePID(pidy, actTime, (0.54 - Quad->IMUC.B_averageDistance) * 5);
    updatePID(pidz, actTime, targetHeight - Quad->IMUC.B_distance0);

    ctrl.roll_d = (0.5 * pidx->currentValue - 0.5 * pidy->currentValue);
    ctrl.pitch_d = (-0.5 * pidx->currentValue - 0.5 * pidy->currentValue);

    // limiting pitch and yaw to about 7°
    ctrl.roll_d = ctrl.roll_d > maxAngle_deg ? maxAngle_deg : ctrl.roll_d;
    ctrl.roll_d = ctrl.roll_d < -maxAngle_deg ? -maxAngle_deg : ctrl.roll_d;

    ctrl.pitch_d = ctrl.pitch_d > maxAngle_deg ? maxAngle_deg : ctrl.pitch_d;
    ctrl.pitch_d = ctrl.pitch_d < -maxAngle_deg ? -maxAngle_deg : ctrl.pitch_d;

    double scaling = 1000.0 / 1.0;

    // feed forward to overcome gravity --> acquired from measurement data thrust0 = 104 // with more sensors (weight) it's 109
    unsigned char thrust0 = 104;  // TODO: gravity scaling 1/cos(phi)/cos(theta), TODO: battery power compensation

    // assign pid to u_thrust only if positive
    double thrust = (pidz->currentValue + thrust0) >= 0 ? pidz->currentValue + thrust0 : 1;
    ctrl.u_thrust = thrust < 200 ? thrust : 200;

    // calculate yaw command
    int maxDelta = 5;  // 5 degrees max

    int tmp_angle = data.angle_yaw - (int)Quad->IMUC.angle_yaw;

    if (Quad->IMUC.angle_yaw > maxDelta) {
        tmp_angle = data.angle_yaw - maxDelta;
    } else if (Quad->IMUC.angle_yaw <= -maxDelta) {
        tmp_angle = data.angle_yaw + maxDelta;
    }
    ctrl.yaw_d = 0.05 * ctrl.yaw_d + 0.95 * tmp_angle;
    // ctrl.yaw_d = tmp_angle;
    ctrl.roll_d *= scaling;
    ctrl.pitch_d *= scaling;

    return 0;
}

//PID Controller for a hovering flight with onboard drone controller
int calculateHover(double I_safeX0, double I_safeY0, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, PID* pidz) {
    double g = 9.81;  // m/s^2
    double m = 1.2;   // "kg"
    double x_ddot, y_ddot;

    double maxAngle = maxAngle_deg * M_PI / 180.0;  // in degree to rad
    double actTime = get_time_ms();

    // path planning
    double outputX[4] = {};
    double outputY[4] = {};
    double outputZ[4] = {};
    double t, t0, a_max, v_max, s_d;
    t = (actTime - Quad->quadStartTime) / 1000.0;
    t0 = 12;       // s
    a_max = 0.25;  // m/s^2
    v_max = 0.15;   // m/s

    continousPath(outputX, 0, 0, a_max, v_max, s_d);
    double trajTime = 6 + 4;  //outputX[3] + 4;

    // Square Trajectory
    s_d = 0.5;  // m // Square dimension

    if (t < t0) {  // takeoff trajectory
        continousPath(outputZ, t, 0, a_max, v_max, 0.255);
        Quad->trajectory.I_z = 245 + (int)(outputZ[2] * 1000);
    } else if (t < t0 + 2 * trajTime) {  // perform rectangular trajectory
        continousPath(outputX, t, t0, a_max, v_max, s_d);
        continousPath(outputY, t, t0 + trajTime, a_max, v_max, s_d);
        Quad->trajectory.I_x = I_safeX0 + (int)(outputX[2] * 1000);
        Quad->trajectory.I_y = I_safeY0 + (int)(outputY[2] * 1000);
    } else {
        continousPath(outputX, t, t0 + 2 * trajTime, a_max, v_max, -s_d);
        continousPath(outputY, t, t0 + 3 * trajTime, a_max, v_max, -s_d);
        outputX[2] += s_d;
        outputY[2] += s_d;
        Quad->trajectory.I_x = I_safeX0 + (int)(outputX[2] * 1000) + s_d;
        Quad->trajectory.I_y = I_safeY0 + (int)(outputY[2] * 1000) + s_d;
    }

    // PID Controller for position and speed correction of quadrotor
    updatePID_statespace(pidx, actTime, (Quad->trajectory.I_x - Quad->I_x_kal) / 1000.0, outputX[1] - Quad->I_x_dot_kal / 1000.0);
    updatePID_statespace(pidy, actTime, (Quad->trajectory.I_y - Quad->I_y_kal) / 1000.0, outputY[1] - Quad->I_y_dot_kal / 1000.0);

    updatePID_statespace(pidz, actTime, (Quad->trajectory.I_z - Quad->I_z_kal) / 1000.0, outputZ[1] - Quad->I_z_dot_kal / 1000.0);

    // calculate motor control from required force calculated by PID_z // Leitner2017
    double F_0 = 3.01;        // 2.5N, for a common thrust of 96
                             // feed forward to overcome gravity --> acquired from measurement u = 104 :-: 109
    double F =  pidz->currentValue;
    double d_F = 0.202 * 0;  // 0.202 N from identification
    double k_F = 8.5041e-4;
    double p = 7.0 / 4.0;
    double u_mot;
    if (F + F_0 - d_F > 0.01) {
        u_mot = pow((F + F_0  - d_F) / k_F, 1.0 / p);
    } else {
        u_mot = 0;
    }

    // assign pid to u_thrust only if positive
    double thrust = (u_mot ) > 0 ? u_mot: 0;
    ctrl->u_thrust = thrust < 200 ? thrust : 200;

    // ctrl->u_thrust = thrust0;
    // printf("u_thrust,%u,", ctrl->u_thrust);

    // assign acceleration from trajectory planner
    x_ddot = outputX[0] + pidx->currentValue;
    y_ddot = outputY[0] + pidy->currentValue;

    // calculate angles in order to move quadrocopter in KI_(xy)-plane

    // asin() is only defined for range [-1,1] therefore limiting is needed
    x_ddot = x_ddot < 1 ? x_ddot : 1;
    x_ddot = x_ddot > -1 ? x_ddot : -1;

    y_ddot = y_ddot < 1 ? y_ddot : 1;
    y_ddot = y_ddot > -1 ? y_ddot : -1;

    // conversion from acceleration to angle, maple script available
    double q6 = Quad->yaw;
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
    ctrl->yaw_d = (210);

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
// takeoff and perform lemniscate trajectory
int lemniscateHover(double I_safeX0, double I_safeY0, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, PID* pidz) {
    double g = 9.81;  // m/s^2
    double m = 1.2;   // "kg"
    double x_ddot, y_ddot;

    double maxAngle = maxAngle_deg * M_PI / 180.0;  // in degree to rad
    double actTime = get_time_ms();

    // path planning
    double outputX[4] = {};
    double outputY[4] = {};
    double outputZ[4] = {};
    double t, t0, a_max, v_max, s_d;
    t = (actTime - Quad->quadStartTime) / 1000.0;
    t0 = 10;       // s
    a_max = 0.25;  // m/s^2
    v_max = 0.15;   // m/s

    if (t < t0) {  // takeoff trajectory
        continousPath(outputZ, t, 0, a_max, v_max, 0.255);
        Quad->trajectory.I_z = 245 + (int)(outputZ[2] * 1000);
    } else if (t <= t0 + M_PI * 3 * 4) {  // perform lemniscate trajectory
        lemniscatePathX(outputX, t, t0);
        lemniscatePathY(outputY, t, t0);
        Quad->trajectory.I_x = I_safeX0 + (int)(outputX[2] * 1000);
        Quad->trajectory.I_y = I_safeY0 + (int)(outputY[2] * 1000);
    } else {
        Quad->trajectory.I_x = I_safeX0;
        Quad->trajectory.I_y = I_safeY0;
    }

    // PID Controller for position and speed correction of quadrotor
    updatePID_statespace(pidx, actTime, (Quad->trajectory.I_x - Quad->I_x_kal) / 1000.0, outputX[1] - Quad->I_x_dot_kal / 1000.0);
    updatePID_statespace(pidy, actTime, (Quad->trajectory.I_y - Quad->I_y_kal) / 1000.0, outputY[1] - Quad->I_y_dot_kal / 1000.0);

    updatePID_statespace(pidz, actTime, (Quad->trajectory.I_z - Quad->I_z_kal) / 1000.0, outputZ[1] - Quad->I_z_dot_kal / 1000.0);

   // calculate motor control from required force calculated by PID_z // Leitner2017
    double F_0 = 3.01;        // 2.5N, for a common thrust of 96
                             // feed forward to overcome gravity --> acquired from measurement u = 104 :-: 109
    double F =  pidz->currentValue;
    double d_F = 0.202 * 0;  // 0.202 N from identification
    double k_F = 8.5041e-4;
    double p = 7.0 / 4.0;
    double u_mot;
    if (F + F_0 - d_F > 0.01) {
        u_mot = pow((F + F_0  - d_F) / k_F, 1.0 / p);
    } else {
        u_mot = 0;
    }

    // assign pid to u_thrust only if positive
    double thrust = (u_mot ) > 0 ? u_mot: 0;
    ctrl->u_thrust = thrust < 200 ? thrust : 200;

    // assign acceleration from trajectory planner
    x_ddot = outputX[0] + pidx->currentValue;
    y_ddot = outputY[0] + pidy->currentValue;

    // calculate angles in order to move quadrocopter in KI_(xy)-plane

    // asin() is only defined for range [-1,1] therefore limiting is needed
    x_ddot = x_ddot < 1 ? x_ddot : 1;
    x_ddot = x_ddot > -1 ? x_ddot : -1;

    y_ddot = y_ddot < 1 ? y_ddot : 1;
    y_ddot = y_ddot > -1 ? y_ddot : -1;

    // conversion from acceleration to angle, maple script available
    double q6 = Quad->yaw;
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
    double yaw_desired = 210; // degrees

    if (fabs(ctrl->roll_d) > 32760 || fabs(ctrl->pitch_d) > 32760) {
        ctrl->roll_d = 0;
        ctrl->pitch_d = 0;
    }

    // assign yaw command
    int yaw_delta = yaw_desired - data.angle_yaw;

    if (yaw_delta > maxDelta) {
        ctrl->yaw_d = data.angle_yaw + maxDelta;
    } else if (yaw_delta < -maxDelta) {
        ctrl->yaw_d = data.angle_yaw - maxDelta;
    }else{
        ctrl->yaw_d = yaw_desired;
    }

    return 0;
}
int lemniscatePathX(double output[], double t, double t0) {
    if (t > t0) {
        output[0] = -1.0 / 9.0 * 1.0 / 2.0 * sin((t - t0) / 3); /*acceleration*/
        output[1] = 1.0 / 3.0 * 1.0 / 2.0 * cos((t - t0) / 3);  /*speed*/
        output[2] = 1.0 / 2.0 * sin((t - t0) / 3);              /*position*/
    }
    output[3] = t0 + M_PI * 3 * 4; /* time */
}
int lemniscatePathY(double output[], double t, double t0) {
    if (t > t0) {
        output[0] = -1.0 / 36.0 * 1.0 / 2.0 * sin((t - t0) / 6); /*acceleration*/
        output[1] = 1.0 / 6.0 * 1.0 / 2.0 * cos((t - t0) / 6);  /*speed*/
        output[2] = 1.0 / 2.0 * sin((t - t0 ) / 6);               /*position*/
    }
    output[3] = t0 + M_PI * 3 * 4; /* time */
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
