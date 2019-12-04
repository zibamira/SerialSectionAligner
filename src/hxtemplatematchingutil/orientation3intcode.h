#ifndef ORIENTATION3_INT_CODE_H
#define ORIENTATION3_INT_CODE_H

#include "api.h"

#include <mclib/McVec3.h>

namespace orientation3intcode {

struct Orientation {
    Orientation() : phi(0), theta(0) {}

    Orientation(unsigned short phi, unsigned short theta)
        : phi(phi), theta(theta) { }

    unsigned short phi;
    unsigned short theta;
};

HXTEMPLATEMATCHINGUTIL_API unsigned int encode(Orientation o);

HXTEMPLATEMATCHINGUTIL_API Orientation decode(unsigned int code);

HXTEMPLATEMATCHINGUTIL_API McVec3f decodeAsVector(unsigned int code);

HXTEMPLATEMATCHINGUTIL_API McVec3f asVector(Orientation o);

HXTEMPLATEMATCHINGUTIL_API McVec3f orientationVector(float phiDeg, float thetaDeg);

}  // namespace orientation3intcode

#endif
