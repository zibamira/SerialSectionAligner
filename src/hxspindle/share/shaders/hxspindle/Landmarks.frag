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




varying vec4 color;
varying vec3 inner;
varying vec4 sphere;


uniform vec2 window;


void main (void)
{
    vec4 col = color;

    float pixSize = abs(2.0 / gl_ProjectionMatrix[0][0]) / window.x;

    vec3  i = window.x * inner;
    float a = 0.0; //1.0 * sqrt(2.0);
    float r = i.z - a;
    float d = sqrt(i.x * i.x + i.y * i.y) - r;
    float b = 5.0;

    if (d > a) // outside circle
    {
        discard;
    }
    /*else if (d > -a) // antialiasing
    {
        //col.w = 1.0 - (abs(d) / i.z);
        col.w = clamp(1.0 - ((d + a) / (2.0 * a)), 0.0, 1.0);
        //col.xyz = vec3(1.0 - ((d + a) / (2.0 * a)), 1.0 - ((d + a) / (2.0 * a)), 1.0 - ((d + a) / (2.0 * a)));
    }*/
    else if (d < -b) // idea 1 make circle inner transparent
    {
        //if (abs(i.x) > 2.0 && abs(i.y) > 2.0)
        {
            col.w = 1.0 - ((abs(d) - b) / (r - b));
        }
    }


    gl_FragColor = vec4(col);

}