/* .name: planTrajectory.c
 * .description: Plan point to point path with sin^2(t) acceleration profile.
 * .usage: 
 * .author: Richard Fischereder
 * .licence: MIT 
 
* Copyright <2017> <Richard Fischereder>

* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <math.h>

// slowest possible path is 2*pi (6.28) seconds long, when using time scaled as 1s per second. 
// If faster paths need to be generated a path parameter sigma can be used
// t = sigma*t --> t_end = t/sigma^2

// plan point to point path with sin^2(t) acceleration
int PTP(double t, double t0, double a_max, double v_max, double s_d, double* y) {
    
    double tcons = 0;

    double accel = (2.0 * s_d) / (pow(M_PI, 2)); /*for continous acceleration*/

    double a = (accel < a_max) ? accel : a_max; /*set a to allowed range*/
    double v = 1.0 / 2.0 * a * M_PI;            /*calculate speed*/
    double s = 1.0 / 2.0 * pow(M_PI, 2) * a;    /*calculate resulting distance*/

    if (v >= v_max) { /*const speed*/
        v = v_max;
        a = (2.0 * v_max) / M_PI;
        tcons = (s_d - 1.0 / 2.0 * pow(M_PI, 2) * a) / (v);
    } else if (s < s_d) { /*const speed*/
        tcons = (s_d - 1.0 / 2.0 * pow(M_PI, 2) * a) / (v);
    }

    double f1, f2, f3, f4, a1, a2;

    /*definition of areas*/
    f1 = (t >= t0 && t < M_PI + t0) ? 1 : 0;
    f2 = (t >= M_PI + t0 && t <= M_PI + tcons + t0) ? 1 : 0;
    f3 = (t > M_PI + tcons + t0 && t <= 2 * M_PI + tcons + t0) ? 1 : 0;
    f4 = (t >= 2 * M_PI + tcons + t0) ? 1 : 0;

    a1 = a * pow(sin(t - t0), 2);
    a2 = a * pow(sin(t - tcons - t0), 2);

    y[0] = a1 * f1 - a2 * f3; /*a*/

    double v1 = -((s_d - tcons * v) * (cos(t - t0) * sin(t - t0) - (t - t0))) / (pow(M_PI, 2)) * f1;
    double v2 = v * f2;
    double v3 = ((s_d - tcons * v) * (cos(t - t0 - tcons) * sin(t - t0 - tcons) + 2 * M_PI - (t - t0 - tcons))) / (pow(M_PI, 2)) * f3;
    double v4 = 0;

    y[1] = v1 + v2 + v3 + v4; /*v*/

    double s1 = 1.0 / 2.0 * ((s_d - tcons * v) * (pow(cos(t - t0), 2) + pow(t - t0, 2) - 1)) / (pow(M_PI, 2)) * f1;
    double s2 = 1.0 / 4.0 * pow(M_PI, 2) * a * f2 + (v * (t - t0 - M_PI)) * f2;
    double bcons = (tcons > 0) ? 1 : 0;
    double s3 = bcons * (s_d - 1.0 / 2.0 * pow(M_PI, 2) * a) * f3 - 1.0 / 2.0 * ((s_d - tcons * v) * (pow(cos(t - t0 - tcons), 2) + 2 * pow(M_PI, 2) - 4 * M_PI * (t - t0 - tcons) + pow(t - t0 - tcons, 2) - 1)) / (pow(M_PI, 2)) * f3;
    double s4 = s_d * f4;
    y[2] = s1 + s2 + s3 + s4; /*s*/

    y[3] = t0 + tcons + 2 * M_PI; /*T*/

    return 0;
}