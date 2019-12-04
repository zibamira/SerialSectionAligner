#version 120
#extension GL_EXT_geometry_shader4 : enable



// input per vertex

varying in vec2 color[];

// output per vertex

varying out vec2  Color;
varying out vec3  RayStart;
varying out vec4  Sphere;




/*
  Computation of a tight square for
  raycasting a sphere.
*/


void main(void)
{
    vec3 m = gl_PositionIn[0].xyz;

    float r = gl_PositionIn[0].w;

    vec3 n_x  = r * vec3(1.0, 0.0, 0.0);
    vec3 n_y  = r * vec3(0.0, 1.0, 0.0);

    Color       = color[0];
    RayStart    = -n_x - n_y;
    Sphere      = gl_PositionIn[0];
    gl_Position = gl_ProjectionMatrix * vec4(m - n_x - n_y, 1.0);
    EmitVertex();

    Color       = color[0];
    RayStart    = -n_x + n_y;
    Sphere      = gl_PositionIn[0];
    gl_Position = gl_ProjectionMatrix * vec4(m - n_x + n_y, 1.0);
    EmitVertex();

    Color       = color[0];
    RayStart    = n_x - n_y;
    Sphere      = gl_PositionIn[0];
    gl_Position = gl_ProjectionMatrix * vec4(m + n_x - n_y, 1.0);
    EmitVertex();

    Color       = color[0];
    RayStart    = n_x + n_y;
    Sphere      = gl_PositionIn[0];
    gl_Position = gl_ProjectionMatrix * vec4(m + n_x + n_y, 1.0);
    EmitVertex();

    EndPrimitive();
}
