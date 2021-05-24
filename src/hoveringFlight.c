#include "../include/hoveringFlight.h"

#include <math.h>
#include <stdio.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/pid.h"
#include "../include/quad.h"

// PID Controller for a hovering flight -- this should work at some point...
int calculateHover(double height, double I_safeX, double I_safeY, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, PID* pidz, double actTime) {
    double g = 9.81;  // m/s^2
    double m = 1.6;   // kg
    double x_ddot, y_ddot;

    double maxAngle = maxAngle_deg * M_PI / 180.0;  // in grad to rad

    updatePID(pidx, actTime, (I_safeX - Quad->I_x) / 1000.0);
    updatePID(pidy, actTime, (I_safeY - Quad->I_y) / 1000.0);
    updatePID(pidz, actTime, (height - Quad->I_z) / 1000.0);

    // asin() is only defined for range [-1,1] therefore limiting is needed
    x_ddot = pidx->currentValue < 1 ? pidx->currentValue : 1;
    x_ddot = x_ddot > -1 ? x_ddot : -1;

    y_ddot = pidy->currentValue < 1 ? pidy->currentValue : 1;
    y_ddot = y_ddot > -1 ? y_ddot : -1;

    // feed forward to overcome gravity --> acquired from measurement data thrust0 = 102

    unsigned char thrust0 = /*m * (g - z_ddot)*/ 102;/* / (cos(Quad->roll) * cos(Quad->pitch));*/  // TODO: get second derivative

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

    // assign to quadrocopter
    // +/- 52° = -2048..2048
    // double scaling = 2048.0 / 52.0;

    // 1 deg == 1000 cts
    double scaling = 1000.0 / 1.0;
    // roll_d = roll_d * 0.85 + 0.15 * (ctrl->roll_d/(scaling * 180.0 / M_PI)) ;
    // pitch_d = pitch_d * 0.85 + 0.15 * (ctrl->roll_d/(scaling * 180.0 / M_PI)) ;

    ctrl->roll_d = (short)((roll_d * (scaling * 180.0 / M_PI)));
    ctrl->pitch_d = (short)((pitch_d * (scaling * 180.0 / M_PI)));

    int maxDelta = 5000;  // 10 degrees max
    ctrl->yaw_d = (210 - 45) * 1000;

    int yaw_delta = ctrl->yaw_d - data.angle_yaw * 1000;

    if (yaw_delta > maxDelta) {
        ctrl->yaw_d = data.angle_yaw * 1000 + maxDelta;
    } else if (yaw_delta < -maxDelta) {
        ctrl->yaw_d = data.angle_yaw * 1000 - maxDelta;
    }

    printf("I_x,%6.2f,I_y,%6.2f,I_z,%6.2f,I_x_dot,%6.2f,I_y_dot,%6.2f,I_z_dot,%6.2f,roll_cmd,%5d,pitch_cmd,%5d,yaw_cmd,%5d,\n", Quad->I_x, Quad->I_y, Quad->I_z, Quad->I_x_dot_kal, Quad->I_y_dot_kal, Quad->I_z_dot_kal, ctrl->roll_d, ctrl->pitch_d, ctrl->yaw_d);

    return 0;
}
