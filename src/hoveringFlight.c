#include "../include/hoveringFlight.h"

#include <math.h>
#include <stdio.h>

#include "../../../asctec-sdk3.0-archive/serialcomm.h"
#include "../include/quad.h"

// P Controller for a hovering flight -- this should work at some point...
int calculateHover(double height, struct QuadState* Quad, struct CONTROL* ctrl) {
    double g = 9.81;  // m/s^2
    double x_ddot, y_ddot;

    // TODO: I_safeX, I_safeY need to be set automatically
    double I_safeX = -575.61;  // mm
    double I_safeY = -52.03;     // mm

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

    double maxAngle = 2.5 * M_PI / 180.0;  // in grad to rad

    // only temporary for hovering
    x_ddot = (I_safeX - I_x) / 125;
    y_ddot = (I_safeY - I_y) / 125;

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

    ctrl->yaw_delta = (int)(q6 * (scaling * 180.0 / M_PI));
    ctrl->yaw_delta = ctrl->yaw_delta > 1000 ? 1000 : ctrl->yaw_delta;    // limit angles
    ctrl->yaw_delta = ctrl->yaw_delta < -1000 ? -1000 : ctrl->yaw_delta;  // limit angles

    printf("I_x: %6.2f,\t I_y: %6.2f,\t I_z: %6.2f,\t snd_cmd: %1d,\t roll_cmd: %5d,\t pitch_cmd: %5d,\tyaw_meas: %5d,\t\n", I_x, I_y, I_z, send_cmd, ctrl->roll_d, ctrl->pitch_d, ctrl->yaw_delta);

    ctrl->roll_d *= send_cmd;
    ctrl->pitch_d *= send_cmd;

    return 0;
}
