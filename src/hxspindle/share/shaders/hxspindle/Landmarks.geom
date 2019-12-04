/**
    Raycasting of a sphere.
    ========================

    This shader raycasts a sphere for orthographic projection
    and cuts it with a slice.

    input: sphere position      = gl_Vertex.xyz
           color                = gl_Color;
           radius               = uniform float "radius"
           slice                = uniform vec4 "slice"
           slice height         = uniform float "sliceHeight"


    Vertex Shader:

    Applies the modelview matrix to the spher position.

    
    Geometry Shader:

    Computes a thight square for the sphere and computes
    the ray start and direction for raycasting.


    Fragment Shader:

    Computes the intersection point of the ray and the
    sphere. Compares the point with the slice and 
    computes the intersection with the slice if necessary.

    author: Norbert Lindow
*/




#version 120
#extension GL_EXT_geometry_shader4 : enable



varying vec4 color;
varying vec3 inner;
varying vec4 sphere;


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

    r = length(gl_ProjectionMatrix * vec4(n_x, 0.0));

    sphere      = gl_PositionIn[0];
    inner       = vec3(-r, -r, r);
    color       = gl_TexCoordIn[0][0];
    gl_Position = gl_ProjectionMatrix * vec4(m - n_x - n_y, 1.0);
    EmitVertex();

    sphere      = gl_PositionIn[0];
    inner       = vec3(-r, r, r);
    color       = gl_TexCoordIn[0][0];
    gl_Position = gl_ProjectionMatrix * vec4(m - n_x + n_y, 1.0);
    EmitVertex();

    sphere      = gl_PositionIn[0];
    inner       = vec3(r, -r, r);
    color       = gl_TexCoordIn[0][0];
    gl_Position = gl_ProjectionMatrix * vec4(m + n_x - n_y, 1.0);
    EmitVertex();

    sphere      = gl_PositionIn[0];
    inner       = vec3(r, r, r);
    color       = gl_TexCoordIn[0][0];
    gl_Position = gl_ProjectionMatrix * vec4(m + n_x + n_y, 1.0);
    EmitVertex();

    EndPrimitive();
}