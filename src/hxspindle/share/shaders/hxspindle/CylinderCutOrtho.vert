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



uniform float radius;

varying vec4 cylinderCol;

void main (void)
{
    vec3  p0   = gl_Vertex.xyz;
    vec3  p1   = gl_MultiTexCoord0.xyz;
    float r    = radius;

    vec4 pT0   = gl_ModelViewMatrix * vec4(p0, 1.0);
    vec4 pT1   = gl_ModelViewMatrix * vec4(p1, 1.0);

    gl_Position    = vec4((1.0 / pT0.w) * pT0.xyz, r);
    gl_TexCoord[0] = vec4((1.0 / pT1.w) * pT1.xyz, 1.0);
    cylinderCol    = gl_Color;
}