/**
    Raycasting of a cylinder.
    ========================

    This shader raycasts a cylinder for orthographic projection
    and cuts it with a slice.

    input: cylinder start point = gl_Vertex.xyz
           cylinder end point   = gl_MultiTexCoord0.xyz
           color                = gl_Color;
           radius               = uniform float "radius"
           slice                = uniform vec4 "slice"
           slice height         = uniform float "sliceHeight"


    Vertex Shader:

    Applies the modelview matrix to the end points of the
    cylinder. 

    
    Geometry Shader:

    Computes a thight square for the cylinder and the
    ray start and direction for raycasting.


    Fragment Shader:

    Computes the intersection points of the ray and the
    cylinder. Compares the points with the slice and 
    computes the intersection with the slice if necessary.

    author: Norbert Lindow
*/



#version 120
#extension GL_EXT_geometry_shader4 : enable

varying in vec4 cylinderCol[];

varying vec4 cylinder;
varying vec4 color;
varying vec4 rayStart;
varying vec4 rotation;



/*
    Quaternion multiplikation.
*/
vec4 qMul(vec4 q1, vec4 q2)
{
    return vec4((q1.w * q2.xyz) + (q2.w * q1.xyz) + cross(q1.xyz, q2.xyz), q1.w * q2.w - dot(q1.xyz, q2.xyz));
}




/*
    Point rotation with quaternion. Axis must have unit length.
*/
vec3 rotatePoint (vec3 point, vec3 axis, float angle)
{
    vec4 q  = vec4(sin(angle / 2.0) * axis, cos(angle / 2.0));

    vec4 qC = vec4(-1.0 * q.xyz, q.w);
    vec4 x  = vec4(point, 0.0);
    vec4 res = qMul(qMul(q, x), qC);

    return res.xyz;
}




/*
    Returns the angle between two vectors.
*/
float angle(vec3 x, vec3 y)
{
    return acos(dot(x,y) / (length(x) * length(y)));
}




/*
  Computation of a tight square for a cylinder.
*/
void main(void)
{
    vec3  p0    = gl_PositionIn[0].xyz;
    vec3  p1    = gl_TexCoordIn[0][0].xyz;

    vec3  m     = 0.5 * (p0 + p1);
    vec3  axis  = 0.5 * (p1 - p0);
    float r     = gl_PositionIn[0].w;
    float h     = length(axis);

    vec3 z_axis = vec3(0.0, 0.0, 1.0);
    vec3 n_y    = normalize(cross(z_axis, axis));
    vec3 n_z    = normalize(cross(axis, n_y));

    float s     = -r;

    if (angle(axis, z_axis) < 1.5707963) s = -s;

    vec3  y_axis    = vec3(0.0, 1.0, 0.0);
    vec3  rot_axis  = normalize(cross(axis, y_axis));
    float rot_angle = angle(axis, y_axis);
    vec4  rot       = vec4(0.0, 0.0, 1.0, 0.0);

    if (rot_axis == rot_axis) rot = vec4(rot_axis, rot_angle);

    vec3 rn_y  = rotatePoint(n_y,  rot.xyz, rot.w);
    vec3 rn_z  = rotatePoint(n_z,  rot.xyz, rot.w);

    // generate quad

    color           = cylinderCol[0];
    cylinder        = vec4(m, r);
    rayStart        = vec4(vec3(0.0, -h, 0.0) + s * rn_z + r * rn_y, h);
    rotation        = rot;

    gl_Position = gl_ProjectionMatrix * vec4(m - axis + s * n_z + r * n_y, 1.0);
    EmitVertex();

    color           = cylinderCol[0];
    cylinder        = vec4(m, r);
    rayStart        = vec4(vec3(0.0, -h, 0.0) + s * rn_z - r * rn_y, h);
    rotation        = rot;
    
    gl_Position = gl_ProjectionMatrix * vec4(m - axis + s * n_z - r * n_y, 1.0);
    EmitVertex();

    color           = cylinderCol[0];
    cylinder        = vec4(m, r);
    rayStart        = vec4(vec3(0.0, h, 0.0) - s * rn_z + r * rn_y, h);
    rotation        = rot;
    
    gl_Position = gl_ProjectionMatrix * vec4(m + axis - s * n_z + r * n_y, 1.0);
    EmitVertex();

    color           = cylinderCol[0];
    cylinder        = vec4(m, r);
    rayStart        = vec4(vec3(0.0, h, 0.0) - s * rn_z - r * rn_y, h);
    rotation        = rot;

    gl_Position = gl_ProjectionMatrix * vec4(m + axis - s * n_z - r * n_y, 1.0);
    EmitVertex();

    EndPrimitive();
}
