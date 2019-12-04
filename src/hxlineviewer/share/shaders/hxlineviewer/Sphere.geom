#version 120
#extension GL_EXT_geometry_shader4 : enable

#include "Geometry.geom"


varying in vec4 color[];

varying out vec4 colorValue;



/*
  Computation of a tight square for
  raycasting a sphere.
*/


void main(void)
{
    vec3 c = cameraPositionInViewSpace();

    vec3  m   = gl_PositionIn[0].xyz;
    vec3  mc  = m - c;

    float r   = gl_PositionIn[0].w;
    float r2  = r * r;
    float d   = length(mc);
    float d2  = dot(mc, mc);
    float s2  = d2 - r2;

    float p   = r2 / d;
    float q   = s2 / d;

    float rad = sqrt(p * q);

    vec3 n_x  = rad * normalize(cross(mc, vec3(0.3, 0.7, 0.8)));
    vec3 n_y  = rad * normalize(cross(mc, n_x));

    vec3 mcr  = (q / (p + q)) * mc;

    m = c + mcr;

    gl_TexCoord[0] = vec4(mc, r);
    gl_TexCoord[1] = vec4(mcr - n_x - n_y, 1.0);
    gl_Position = gl_ProjectionMatrix * vec4(m - n_x - n_y, 1.0);

    colorValue = color[0];

    EmitVertex();

    gl_TexCoord[0] = vec4(mc, r);
    gl_TexCoord[1] = vec4(mcr - n_x + n_y, 1.0);
    gl_Position = gl_ProjectionMatrix * vec4(m - n_x + n_y, 1.0);

    colorValue = color[0];

    EmitVertex();

    gl_TexCoord[0] = vec4(mc, r);
    gl_TexCoord[1] = vec4(mcr + n_x - n_y, 1.0);
    gl_Position = gl_ProjectionMatrix * vec4(m + n_x - n_y, 1.0);

    colorValue = color[0];

    EmitVertex();

    gl_TexCoord[0] = vec4(mc, r);
    gl_TexCoord[1] = vec4(mcr + n_x + n_y, 1.0);
    gl_Position = gl_ProjectionMatrix * vec4(m + n_x + n_y, 1.0);

    colorValue = color[0];

    EmitVertex();

    EndPrimitive();
}
