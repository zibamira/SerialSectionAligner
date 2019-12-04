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
varying out vec3 Ray;
varying out vec4 Rotation;

/**
  Main: computation of a tight square for a cylinder.
*/
void main(void)
{

    vec3 c = cameraPositionInViewSpace();

    vec3  m   = gl_PositionIn[0].xyz - c;
    float r   = gl_PositionIn[0].w;
    vec3  axe = cylinder[0];

    vec3 n_y  = normalize(cross(m, axe));
    vec3 n_z  = normalize(cross(axe, n_y));

    // compute sphere height
    float d2  = min(dot(m + axe, m + axe) , dot(m - axe, m - axe));
    float d   = sqrt(d2);
    float r2  = r * r;
    float h   = sqrt(r2 + ((r2 * r2) / (d2 - r2)));

    // compute left vertex and the height
    vec3 v_1 = m - axe;
    vec3 vf  = v_1 - r * n_z;
    vec3 vb  = v_1 + r * n_z;

    float s     = 0;
    float s_1ny = 1.0;
    float s_2ny = 1.0;

    if (dot(n_y, cross(vf, vb)) > 0.0)
    {
        v_1 = vf;
        
        if (dot(cross(v_1, v_1 + 2.0 * axe), n_y) < 0.0)
        {
            v_1 = v_1 + 2.0 * axe;
        }
    }
    else
    {
        v_1 = vb;
    }

    s = (dot(v_1, v_1) - d2 - r2) / (2.0 * d);

    s_1ny = (h * (d + s)) / d;


    // compute right vertex and height

    vec3 v_2  = m + axe;
         vf   = v_2 - r * n_z;
         vb   = v_2 + r * n_z;

    if (dot(n_y, cross(vf, vb)) < 0.0)
    {
        v_2 = vf;

        if (dot(cross(v_2, v_2 - 2.0 * axe), n_y) > 0.0)
        {
            v_2 = v_2 - 2.0 * axe;
        }
    }
    else
    {
        v_2 = vb;
    }

    s = (dot(v_2, v_2) - d2 - r2) / (2.0 * d);

    s_2ny = (h * (d + s)) / d;


    vec3  yAxe = vec3(0.0, 1.0, 0.0);
    vec4  rotQ = qRot(normalize(cross(axe, yAxe)), angle(axe, yAxe));

    vec3 rv_1 = v_1;
    vec3 rv_2 = v_2;
    vec3 rn_y = n_y;

    vec3 n_c1  = clip1[0].xyz;
    vec3 n_c2  = clip2[0].xyz;

    m    = rotatePoint(m,    rotQ);
    rv_1 = rotatePoint(rv_1, rotQ);
    rv_2 = rotatePoint(rv_2, rotQ);
    rn_y = rotatePoint(rn_y, rotQ);
    n_c1 = rotatePoint(n_c1, rotQ);
    n_c2 = rotatePoint(n_c2, rotQ);

    s = length(axe);

    Clip1          = vec4(n_c1, n_c1.y * -clip1[0].w);
    Clip2          = vec4(n_c2, n_c2.y *  clip2[0].w);
    Colors         = colorValues[0];
    Cylinder       = vec4(m, r);
    Ray            = rv_1 - s_1ny * rn_y;
    Rotation       = rotQ;

    gl_Position = gl_ProjectionMatrix * vec4(c + v_1 - s_1ny * n_y, 1.0);
    EmitVertex();

    Clip1          = vec4(n_c1, n_c1.y * -clip1[0].w);
    Clip2          = vec4(n_c2, n_c2.y *  clip2[0].w);
    Colors         = colorValues[0];
    Cylinder       = vec4(m, r);
    Ray            = rv_1 + s_1ny * rn_y;
    Rotation       = rotQ;

    gl_Position = gl_ProjectionMatrix * vec4(c + v_1 + s_1ny * n_y, 1.0);
    EmitVertex();

    Clip1          = vec4(n_c1, n_c1.y * -clip1[0].w);
    Clip2          = vec4(n_c2, n_c2.y *  clip2[0].w);
    Colors         = colorValues[0];
    Cylinder       = vec4(m, r);
    Ray            = rv_2 - s_2ny * rn_y;
    Rotation       = rotQ;

    gl_Position = gl_ProjectionMatrix * vec4(c + v_2 - s_2ny * n_y, 1.0);
    EmitVertex();

    Clip1          = vec4(n_c1, n_c1.y * -clip1[0].w);
    Clip2          = vec4(n_c2, n_c2.y *  clip2[0].w);
    Colors         = colorValues[0];
    Cylinder       = vec4(m, r);
    Ray            = rv_2 + s_2ny * rn_y;
    Rotation       = rotQ;

    gl_Position = gl_ProjectionMatrix * vec4(c + v_2 + s_2ny * n_y, 1.0);
    EmitVertex();

    EndPrimitive();
}
