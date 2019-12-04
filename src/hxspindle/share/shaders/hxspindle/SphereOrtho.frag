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




varying vec4 color;
varying vec3 rayStart;
varying vec4 sphere;

uniform vec4 view_sphere;





/*
    Returns the intersection point of a ray (r_s + t * r_d)
    and a sphere (m, r).
*/
bool isecSphere(vec3 r_s, vec3 r_d, vec3 m, float r, out vec3 isec1, out vec3 isec2)
{
    vec3 r_st = r_s - m;

    float p = 2.0 * dot(r_st, r_d);
    float q = dot(r_st, r_st) - r * r;
    float w = p * p * 0.25 - q;

    if (w < 0.0) return false;

    float t1 = (-p * 0.5) + sqrt(w);
    float t2 = (-p * 0.5) - sqrt(w);

    isec1 = m + r_st + t1 * r_d;
    isec2 = m + r_st + t2 * r_d;

    return true;
}





void main (void)
{
    // view sphere test

    vec3  v_m = vec3(gl_ModelViewMatrix * vec4(view_sphere.xyz, 1.0));
    float v_r = view_sphere.w;

    vec3  m  = sphere.xyz;
    float r  = sphere.w;
    vec3  rs = rayStart + m;
    vec3  rd = vec3(0.0, 0.0, 1.0);

    vec3 isec1;
    vec3 isec2;
    vec3 view_isec1;
    vec3 view_isec2;

    if (!isecSphere(rs, rd, m, r, isec1, isec2)) discard;

    vec3 isec = isec1;
    vec3 n    = normalize(isec - m);

    if (length(v_m - isec) > v_r)
    {
        isec = isec2;

        if (length(v_m - isec) <= v_r)
        {
            if (isecSphere(rs, rd, v_m, v_r, view_isec1, view_isec2))
            {
                isec = view_isec1;
                n    = normalize(isec - v_m);
            }
        }
        else discard;
    }


    vec4 np = gl_ProjectionMatrix * vec4(isec, 1.0);

    gl_FragDepth = ((np.z / np.w) + 1.0) * 0.5;
    gl_FragColor = vec4(abs(dot(n, rd)) * color.xyz, 1.0);
}