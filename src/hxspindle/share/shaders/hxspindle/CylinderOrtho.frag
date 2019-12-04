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


varying vec4 cylinder;
varying vec4 color;
varying vec4 rayStart;
varying vec4 rotation;


uniform vec4 view_sphere;




/*
    Quaternion multiplikation.
*/
vec4 qMul(vec4 q1, vec4 q2)
{
    return vec4((q1.w * q2.xyz) + (q2.w * q1.xyz) + cross(q1.xyz, q2.xyz), q1.w * q2.w - dot(q1.xyz, q2.xyz));
}




/*
    Point rotation with quaternion. Axe must have unit length.
*/
vec3 rotatePoint (vec3 point, vec3 axis, float angle)
{
    vec4 q  = vec4(sin(angle * 0.5) * axis, cos(angle * 0.5));

    vec4 qC = vec4(-1.0 * q.xyz, q.w);
    vec4 x  = vec4(point, 0.0);
    vec4 res = qMul(qMul(q, x), qC);

    return res.xyz;
}




/*
    Helper function for isecCylinder.
*/
float dotC(vec3 v1, vec3 v2)
{
    return v1.x * v2.x + v1.z * v2.z;
}




/* 
  Returns the intersection points of a ray and a cylinder.
  The ray starts at "rs" and has direction "rd". The midpoint
  of the cylinder is at zero position and the radius is "r".
  The cylinder axis is equal to the Y-axis.
*/
void isecCylinder(vec3 rs, vec3 rd, float r, out vec3 isec_first, out vec3 isec_second)
{
    float w = 1.0 / dotC(rd, rd);
    float p = 2.0 * dotC(rs, rd) * w;
    float q = (dotC(rs, rs) - r * r) * w;

    w = p * p * 0.25 - q;

    if (w < 0.0) discard;

    w = sqrt(w);
    p = -0.5 * p;
    
    isec_first  = rs + (p + w) * rd;
    isec_second = rs + (p - w) * rd;
}




/*
   Returns gradient of a cylinder in zero position and y-axis direction.
*/
vec3 gradCylinder(vec3 p)
{
    return normalize(vec3(-2.0 * p.x, 0.0, -2.0 * p.z));
}




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




/*
    Main.
*/
void main (void)
{
    // view sphere test

    vec3  v_m = vec3(gl_ModelViewMatrix * vec4(view_sphere.xyz, 1.0));
    float v_r = view_sphere.w;

    float r     = cylinder.w;
    float h     = rayStart.w;
    vec3  m     = cylinder.xyz;
    vec3  rd    = rotatePoint(vec3(0.0, 0.0, 1.0), rotation.xyz, rotation.w);
    vec3  rs    = rayStart.xyz;

    // compute intersection with view sphere

    vec3 view_isec1;
    vec3 view_isec2;
    vec3 view_rs = rotatePoint(rs, rotation.xyz, -rotation.w) + m;

    bool view_hit = isecSphere(view_rs, vec3(0.0, 0.0, 1.0), v_m, v_r, view_isec1, view_isec2);

    // compute intersection points with cylinder

    vec3 p_first;
    vec3 p_second;

    isecCylinder(rs, rd, r, p_first, p_second);

    vec3 p  = p_first;
    vec3 n  = gradCylinder(p);
    vec3 pf = rotatePoint(p, rotation.xyz, -rotation.w) + m;

    /*
        if first point is not valid, try intersection point
        with the view sphere
    */
    if (abs(p.y) > h || length(v_m - pf) > v_r)
    {
        if (!view_hit) discard;

        pf = view_isec1;
        p  = rotatePoint(pf - m, rotation.xyz, rotation.w);
        n  = rotatePoint(normalize(pf - v_m), rotation.xyz, rotation.w);

        if (abs(p.y) > h || p.x * p.x + p.z * p.z > r * r) discard;
    }

    vec4 np = gl_ProjectionMatrix * vec4(pf, 1.0);

    gl_FragDepth = ((np.z / np.w) + 1.0) * 0.5; 
    gl_FragColor = vec4(abs(dot(n, rd)) * color.xyz, 1.0);

}