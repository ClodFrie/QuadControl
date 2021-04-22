#include "../include/hoveringFlight.h"

#include <math.h>
#include <stdio.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/kalman.h"
#include "../include/pid.h"
#include "../include/quad.h"

// PID Controller for a hovering flight -- this should work at some point...
int calculateHover(double height, double I_safeX, double I_safeY, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, PID* pidz, double actTime) {
    double g = 9.81;  // m/s^2
    double x_ddot, y_ddot;

    // START critical section
    double q4 = Quad->roll;
    double q5 = Quad->pitch;
    double q6 = Quad->yaw;

    double Qx0 = Quad->Qx;
    double Qy0 = Quad->Qy;
    double Qz0 = Quad->Qz;

    Quad->I_x = cos(q5) * cos(q6) * Qx0 + (sin(q4) * sin(q5) * cos(q6) - cos(q4) * sin(q6)) * Qy0 + (cos(q4) * sin(q5) * cos(q6) + sin(q4) * sin(q6)) * Qz0;
    Quad->I_y = cos(q5) * sin(q6) * Qx0 + (sin(q4) * sin(q5) * sin(q6) + cos(q4) * cos(q6)) * Qy0 + (cos(q4) * sin(q5) * sin(q6) - sin(q4) * cos(q6)) * Qz0;
    Quad->I_z = -sin(q5) * Qx0 + sin(q4) * cos(q5) * Qy0 + cos(q4) * cos(q5) * Qz0;

    // state estimation: kalman filter
    double states[6];
    kalmanFilter(states, Quad->I_x, Quad->I_y, Quad->I_z);
    Quad->I_x = states[0];
    Quad->I_x_dot = states[1];
    Quad->I_y = states[2];
    Quad->I_y_dot = states[3];
    Quad->I_z = states[4];
    Quad->I_z_dot = states[5];

    // END critical section

    double maxAngle = maxAngle_deg * M_PI / 180.0;  // in grad to rad

    // calculate controller output
    updatePID(pidx, actTime, (I_safeX - Quad->I_x) / 1000.0);
    updatePID(pidy, actTime, (I_safeY - Quad->I_y) / 1000.0);
    updatePID_statespace(pidz, actTime, -(height - Quad->I_z) / 1000.0, Quad->I_z_dot / 1000.0);

    // asin() is only defined for range [-1,1] therefore limiting is needed
    x_ddot = pidx->currentValue < 1 ? pidx->currentValue : 1;
    x_ddot = x_ddot > -1 ? x_ddot : -1;

    y_ddot = pidy->currentValue < 1 ? pidy->currentValue : 1;
    y_ddot = y_ddot > -1 ? y_ddot : -1;

    // assign pidz to u_thrust only if positive
    unsigned char thrust0 = 94;  // feed forward to overcome gravity --> acquired from measurement data thrust0 = 102
    double thrust = (pidz->currentValue + thrust0) >= 0 ? pidz->currentValue + thrust0 : 0;
    ctrl->u_thrust = thrust < 200 ? thrust : 200;
    // ctrl->u_thrust = thrust0;
    printf("u_thrust,%u,", ctrl->u_thrust);

    q6 = -q6;  // TODO: fix this -> q6 needs to be in inertial frame

    // calculate angles in order to move quadrocopter in KI_(xy)-plane
    double roll_d = -asin((x_ddot * sin(q6) + y_ddot * cos(q6)) / g);
    double pitch_d = -asin((y_ddot * sin(q6) - x_ddot * cos(q6)) / cos(roll_d) / g);

    roll_d = roll_d > maxAngle ? maxAngle : roll_d;       // limit angles
    roll_d = roll_d < -maxAngle ? -maxAngle : roll_d;     // limit angles
    pitch_d = pitch_d > maxAngle ? maxAngle : pitch_d;    // limit angles
    pitch_d = pitch_d < -maxAngle ? -maxAngle : pitch_d;  // limit angles

    // assign to quadrocopter
    // +/- 52° = -2048..2048
    // double scaling = 2048.0 / 52.0;

    // 1 deg == 1000 cts
    double scaling = 1000.0 / 1.0;
    ctrl->roll_d = (short)((roll_d * (scaling * 180.0 / M_PI)));
    ctrl->pitch_d = (short)((pitch_d * (scaling * 180.0 / M_PI)));

    int maxDelta = 5000;  // 10 degrees max
    ctrl->yaw_d = 180 * 1000;

    int yaw_delta = ctrl->yaw_d - data.angle_yaw * 1000;

    if (yaw_delta > maxDelta) {
        ctrl->yaw_d = data.angle_yaw * 1000 + maxDelta;
    } else if (yaw_delta < -maxDelta) {
        ctrl->yaw_d = data.angle_yaw * 1000 - maxDelta;
    }

    printf("I_x,%6.2f,I_y,%6.2f,I_z,%6.2f,roll_cmd,%5d,pitch_cmd,%5d,yaw_cmd,%5d,\n", Quad->I_x, Quad->I_y, Quad->I_z, ctrl->roll_d, ctrl->pitch_d, ctrl->yaw_d);

    return 0;
}