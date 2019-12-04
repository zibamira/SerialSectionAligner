
#version 120
#extension GL_EXT_geometry_shader4 : enable

#include "Geometry.geom"

varying in vec4 clip1[];
varying in vec4 clip2[];
varying in vec3 colorValues[];
varying in vec3 cylinder[];


varying out vec4 Clip1;
varying out vec4 Clip2;
varying out vec3 Colors;
varying out vec4 Cylinder;
varying out vec3 RayStart;
varying out vec3 RayDirection;
varying out vec4 Rotation;



/**
  Main: computation of a tight square for a cylinder.
*/
void main(void)
{
    vec3  m     = gl_PositionIn[0].xyz;
    vec3  axis  = cylinder[0].xyz;
    float r     = gl_PositionIn[0].w;
    float h     = length(axis);

    vec3 z_axis = vec3(0.0, 0.0, 1.0);
    vec3 n_y    = normalize(cross(z_axis, axis));
    vec3 n_z    = normalize(cross(axis, n_y));

    float s     = -r;

    if (angle(axis, z_axis) < 1.5707963) s = -s;

    vec3  y_axis = vec3(0.0, 1.0, 0.0);
    vec4  rot    = qRot(normalize(cross(axis, y_axis)), angle(axis, y_axis));

    vec3 rn_y    = n_y;
    vec3 rn_z    = n_z;
    vec3 raxis   = axis;
    vec3 rd      = z_axis;
    vec3 n_c1    = clip1[0].xyz;
    vec3 n_c2    = clip2[0].xyz;

    rn_y  = rotatePoint(rn_y,  rot);
    rn_z  = rotatePoint(rn_z,  rot);
    raxis = rotatePoint(raxis, rot);
    rd    = rotatePoint(rd,    rot);
    n_c1  = rotatePoint(n_c1,  rot);
    n_c2  = rotatePoint(n_c2,  rot);

    // output

    Clip1          = vec4(n_c1, n_c1.y * -clip1[0].w);
    Clip2          = vec4(n_c2, n_c2.y *  clip2[0].w);
    Colors         = colorValues[0];
    Cylinder       = vec4(m, r);
    RayStart       = -raxis + s * rn_z + r * rn_y;
    RayDirection   = rd;
    Rotation       = rot;

    gl_Position = gl_ProjectionMatrix * vec4(m - axis + s * n_z + r * n_y, 1.0);
    EmitVertex();

    Clip1          = vec4(n_c1, n_c1.y * -clip1[0].w);
    Clip2          = vec4(n_c2, n_c2.y *  clip2[0].w);
    Colors         = colorValues[0];
    Cylinder       = vec4(m, r);
    RayStart       = -raxis + s * rn_z - r * rn_y;
    RayDirection   = rd;
    Rotation       = rot;

    gl_Position = gl_ProjectionMatrix * vec4(m - axis + s * n_z - r * n_y, 1.0);
    EmitVertex();

    Clip1          = vec4(n_c1, n_c1.y * -clip1[0].w);
    Clip2          = vec4(n_c2, n_c2.y *  clip2[0].w);
    Colors         = colorValues[0];
    Cylinder       = vec4(m, r);
    RayStart       = raxis - s * rn_z + r * rn_y;
    RayDirection   = rd;
    Rotation       = rot;

    gl_Position = gl_ProjectionMatrix * vec4(m + axis - s * n_z + r * n_y, 1.0);
    EmitVertex();

    Clip1          = vec4(n_c1, n_c1.y * -clip1[0].w);
    Clip2          = vec4(n_c2, n_c2.y *  clip2[0].w);
    Colors         = colorValues[0];
    Cylinder       = vec4(m, r);
    RayStart       = raxis - s * rn_z - r * rn_y;
    RayDirection   = rd;
    Rotation       = rot;

    gl_Position = gl_ProjectionMatrix * vec4(m + axis - s * n_z - r * n_y, 1.0);
    EmitVertex();

    EndPrimitive();
}
