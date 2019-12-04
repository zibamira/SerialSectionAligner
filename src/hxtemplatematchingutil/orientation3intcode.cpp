#include "orientation3intcode.h"
#include <mclib/McVec3.h>
#include <mclib/McMat4.h>
#include <mclib/McMath.h>

orientation3intcode::Orientation
orientation3intcode::decode(const unsigned int code)
{
    Orientation o;

    unsigned int temp = code;
    temp /= 65536;
    o.phi = temp;
    temp *= 65536;
    o.theta = code - temp;
    return o;
}

unsigned int orientation3intcode::encode(Orientation o)
{
    return o.phi * 65536 + o.theta;
}

McVec3f orientation3intcode::decodeAsVector(unsigned int code)
{
    return asVector(decode(code));
}

McVec3f orientation3intcode::asVector(Orientation o)
{
    return orientationVector(o.phi, o.theta);
}

McVec3f orientation3intcode::orientationVector(float phiDeg, float thetaDeg)
{
    float phiRad = (phiDeg / 180.0) * M_PI;
    float thetaRad = (thetaDeg / 180.0) * M_PI;
    return McVec3f(
            cos(thetaRad) * cos(phiRad),
            cos(thetaRad) * sin(phiRad),
            -sin(thetaRad));
}
