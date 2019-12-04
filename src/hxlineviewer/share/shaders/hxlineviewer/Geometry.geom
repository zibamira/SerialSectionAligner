
/**
    Returns the angle between two normalized vectors.
*/
float angle(vec3 x, vec3 y)
{
    return acos(dot(x,y) / (length(x) * length(y)));
}




/**
    Quaternion multiplikation.
*/
vec4 qMul(vec4 q1, vec4 q2)
{
    return vec4((q1.w * q2.xyz) + (q2.w * q1.xyz) + cross(q1.xyz, q2.xyz), q1.w * q2.w - dot(q1.xyz, q2.xyz));
}




/**
    Returns a rotation quaternion for a normalized axis
    and an angle.
*/
vec4 qRot(vec3 axis, float angle)
{
    // angle is too small => no rotation
    if (abs(angle) < 0.001) return vec4(0.0, 0.0, 0.0,  1.0);

    // angle is 180° => set axis because axis could be invalid
    else if (angle > 3.14 && angle < 3.143)
    {
        axis = vec3(1.0, 0.0, 0.0);
    }

    return vec4(sin(angle * 0.5) * axis, cos(angle * 0.5));
}




/**
    Rotates a 3d point with a quaternion.
*/
vec3 rotatePoint (vec3 point, vec4 rotQ)
{
    vec4 qC = vec4(-1.0 * rotQ.xyz, rotQ.w);
    vec4 x  = vec4(point, 0.0);
    vec4 res = qMul(qMul(rotQ, x), qC);

    return res.xyz;
}




/**
    Rotates a 3d point around an axis. The axis has to be
    normalized.
*/
vec3 rotatePoint (vec3 point, vec3 axis, float angle)
{
    vec4 q  = vec4(sin(angle / 2.0) * axis, cos(angle / 2.0));

    vec4 qC = vec4(-1.0 * q.xyz, q.w);
    vec4 x  = vec4(point, 0.0);
    vec4 res = qMul(qMul(q, x), qC);

    return res.xyz;
}




/**
    Inverse rotation of a 3d point.
*/
vec3 rotatePointInv (vec3 point, vec4 rotQ)
{
    vec4 qC = vec4(-1.0 * rotQ.xyz, rotQ.w);
    vec4 x  = vec4(point, 0.0);
    vec4 res = qMul(qMul(qC, x), rotQ);

    return res.xyz;
}




/**
    Computes the camera position in model view space.
*/
vec3 cameraPositionInViewSpace()
{
    vec4 cameraPlane = gl_ProjectionMatrixTranspose[3];
    vec4 p_0         = gl_ProjectionMatrixInverse * vec4(-1.0, -1.0, -1.0, 1.0);
    vec4 p_1         = gl_ProjectionMatrixInverse * vec4(-1.0, -1.0,  1.0, 1.0);

    p_0 /= p_0.w;
    p_1 /= p_1.w;

    float t = -dot(p_0, cameraPlane) / dot(p_1 - p_0, cameraPlane);

    return p_0.xyz + t * (p_1.xyz - p_0.xyz);
}
