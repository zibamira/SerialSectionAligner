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



uniform float radius;


void main (void)
{
    //sphere in modelview space 
    gl_Position    = vec4(vec3(gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0)), radius);
    gl_TexCoord[0] = gl_Color;
}

