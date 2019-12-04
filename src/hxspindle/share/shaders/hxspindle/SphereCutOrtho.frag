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
varying vec3 rayStart;
varying vec4 sphere;

uniform vec4  slice;
uniform float sliceHeight;




/*
    Returns the intersection point of a sphere and a plane.
*/
vec3 isecPlane(vec3 pPosition, vec3 pNormal, vec3 rStart, vec3 rDirection)
{
    vec3  rS = rStart - pPosition;
    vec3  n  = pNormal;
    vec3  rD = rDirection;
    float t  = -dot(rS, n) / dot(rD, n);

    return rS + t * rD + pPosition;
}





void main (void)
{
    vec3  rs = rayStart;
    vec3  rd = vec3(0.0, 0.0, 1.0);
    vec3  m  = sphere.xyz;
    float r  = sphere.w;

    // cut
    vec3 c_p = vec3(gl_ModelViewMatrix * vec4(slice.xyz * slice.w, 1.0)) - m;
    vec3 c_n = normalize(vec3(gl_ModelViewMatrix * vec4(slice.xyz, 0.0)));

    c_p = c_p + abs(sliceHeight) * c_n;

    // computes intersection of ray with plane
    vec3 isec = isecPlane(c_p, c_n, rs, rd);
    vec3 n    = c_n;

    if (dot(isec, isec) > r * r)
    {
        float p = 2.0 * dot(rs, rd);
        float q = dot(rs, rs) - r * r;
        float w = p * p * 0.25 - q;

        if (w < 0.0) discard;

        float t = (-p * 0.5) + sqrt(w);

        isec = rs + t * rd;
        n    = normalize(isec);

        float dn = dot(isec - c_p, c_n);

        if (dn > 0.0 || dn < -sliceHeight*2.0) discard;
    }

    vec4 np = gl_ProjectionMatrix * vec4(isec + m, 1.0);

    gl_FragDepth = ((np.z / np.w) + 1.0) * 0.5;
    gl_FragColor = vec4(/*abs(dot(n, rd)) * */ color);
}