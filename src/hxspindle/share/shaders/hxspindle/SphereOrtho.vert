/**
    Raycasting of a sphere.
    ========================

    This shader raycasts a sphere for orthographic projection
    inside a view sphere.

    input: sphere position      = gl_Vertex.xyz
           color                = gl_Color
           radius               = uniform float "radius"
           cut sphere           = uniform vec4 "view_sphere"


    Vertex Shader:

    Applies the modelview matrix to the sphere position.


    Geometry Shader:

    Computes a thight square for the sphere and computes
    the ray start for raycasting.


    Fragment Shader:

    Computes the intersection point of the ray and the
    sphere. Compares the point with the view_sphere and 
    computes the intersection with the view_sphere if necessary.

    author: Norbert Lindow
*/



uniform float radius;


void main (void)
{
    //sphere in modelview space 
    gl_Position    = vec4(vec3(gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0)), radius);
    gl_TexCoord[0] = gl_Color;
}

