#ifndef HOVERINGFLIGHT_H
#define HOVERINGFLIGHT_H

#include "quad.h"
#include "../../../asctec-sdk3.0-archive/serialcomm.h"

int calculateHover(double height, struct QuadState* Quad, struct CONTROL* ctrl);



#endif