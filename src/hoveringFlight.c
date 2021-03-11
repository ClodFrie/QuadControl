#include "../include/hoveringFlight.h"

#include <math.h>
#include <stdio.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/pid.h"
#include "../include/quad.h"

// P Controller for a hovering flight -- this should work at some point...
int calculateHover(double height, double I_safeX, double I_safeY, double maxAngle_deg, struct QuadState* Quad, struct CONTROL* ctrl, PID* pidx, PID* pidy, double actTime) {
    double g = 9.81;  // m/s^2
    double x_ddot, y_ddot;

    // START critical section
    double q4 = Quad->roll;
    double q5 = Quad->pitch;
    double q6 = Quad->yaw;

    double Qx0 = Quad->Qx;
    double Qy0 = Quad->Qy;
    double Qz0 = Quad->Qz;
    // END critical section

    double I_x = cos(q5) * cos(q6) * Qx0 + (sin(q4) * sin(q5) * cos(q6) - cos(q4) * sin(q6)) * Qy0 + (cos(q4) * sin(q5) * cos(q6) + sin(q4) * sin(q6)) * Qz0;
    double I_y = cos(q5) * sin(q6) * Qx0 + (sin(q4) * sin(q5) * sin(q6) + cos(q4) * cos(q6)) * Qy0 + (cos(q4) * sin(q5) * sin(q6) - sin(q4) * cos(q6)) * Qz0;
    double I_z = -sin(q5) * Qx0 + sin(q4) * cos(q5) * Qy0 + cos(q4) * cos(q5) * Qz0;
    // printf("Qx0:%f, Qy0:%f, Qz0:%f", Qx0, Qy0, Qz0);

    double maxAngle = maxAngle_deg * M_PI / 180.0;  // in grad to rad

    // calculate controller output
    // updatePID(pidx, actTime, (I_safeX - I_x)/1000.0);
    // updatePID(pidy, actTime, (I_safeY - I_y)/1000.0);
    updatePID(pidx, actTime, (I_safeX - I_x)/10.0);

    printf("x_ddot:%lf,\ty_ddot:%lf",pidx->currentValue,pidy->currentValue);
    // asin() is only defined for range [-1,1] therfore limitting is needed
    x_ddot = pidx->currentValue < 1 ? pidx->currentValue : 1;
    x_ddot = x_ddot > -1 ? x_ddot : -1;

    y_ddot = pidy->currentValue < 1 ? pidy->currentValue : 1;
    y_ddot = y_ddot > -1 ? y_ddot : -1;



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

    short send_cmd = 1;
    // disable during takeoff
    if (I_z < 400) {
        send_cmd = 1;
    }

    int maxDelta = 5000;  // 10 degrees max
    ctrl->yaw_d = 180 * 1000;

    int yaw_delta = ctrl->yaw_d - data.angle_yaw * 1000;

    if (yaw_delta > maxDelta) {
        ctrl->yaw_d = data.angle_yaw * 1000 + maxDelta;
    } else if (yaw_delta < -maxDelta) {
        ctrl->yaw_d = data.angle_yaw * 1000 - maxDelta;
    }

    printf("I_x: %6.2f,\t I_y: %6.2f,\t I_z: %6.2f,\t snd_cmd: %1d,\t roll_cmd: %5d,\t pitch_cmd: %5d,\tyaw_cmd: %5d,\t\n", I_x, I_y, I_z, send_cmd, ctrl->roll_d, ctrl->pitch_d, ctrl->yaw_d);

    ctrl->roll_d *= send_cmd;
    ctrl->pitch_d *= send_cmd;

    return 0;
}