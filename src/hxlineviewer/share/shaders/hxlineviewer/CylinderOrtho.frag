#include "Colormap.frag"
#include "Geometry.geom"
#include "Light.frag"


// input per fragment

varying vec4 Clip1;
varying vec4 Clip2;
varying vec3 Colors;
varying vec4 Cylinder;
varying vec3 RayStart;
varying vec3 RayDirection;
varying vec4 Rotation;

//input global

uniform float normalInterpolation;
uniform int numClipPlane;



/**
    Helper function for isecCylinder.
*/
float dotCyl(vec3 v1, vec3 v2)
{
    return v1.x * v2.x + v1.z * v2.z;
}




bool clipPoint(vec3 p)
{
    vec3 tp = rotatePointInv(p, Rotation) + Cylinder.xyz;

    for (int i = 0; i < numClipPlane; ++i)
    {
        if (dot(vec4(tp, 1.0), gl_ClipPlane[i]) < 0.0) return true;
    }

    return false;
}




/**
  Returns the intersection point of a ray with a cylinder segment.
  The ray starts at zero position and has direction rd.
  The cylinder has midpoint m. With plane_1 and plane_2,
  the cylinder will cutted.
*/
vec3 isecCylinder(vec3 rs, vec3 rd, float radius, vec4 plane_1, vec4 plane_2)
{
    vec3 res = vec3(0.0, 0.0, 0.0);

    float w = 1.0 / dotCyl(rd, rd);
    float p = 2.0 * dotCyl(rd, rs) * w;
    float q = (dotCyl(rs, rs) - radius * radius) * w;

    w = p * p * 0.25 - q;

    if (w < 0.0) discard;

    res = rs + (-p * 0.5 + sqrt(w)) * rd;

    // first intersection point does not lie in cone segment
    if (dot(plane_1.xyz, res) - plane_1.w > 0.0 ||
        dot(plane_2.xyz, res) - plane_2.w > 0.0 ||
        clipPoint(res))
    {
        res = rs + (-p * 0.5 - sqrt(w)) * rd;

        // second intersection point does not lie in segment
        if (dot(plane_1.xyz, res) - plane_1.w > 0.0 ||
            dot(plane_2.xyz, res) - plane_2.w > 0.0 ||
            clipPoint(res))
        {
            discard;
        }
    }

    return res;
}




/**
   Returns gradient of a cylinder.
*/
vec3 gradCylinder(vec3 p)
{
    return normalize(vec3(-2.0 * p.x, 0.0, -2.0 * p.z));
}




/**
    Main.
*/
void main (void)
{
    // reading data
    vec3  rd     = normalize(RayDirection);
    vec3  rs     = RayStart;
    vec3  m      = Cylinder.xyz;
    float r      = Cylinder.w;

    // compute cone intersection
    vec3  p     = isecCylinder(rs, rd, r, Clip1, Clip2);

    float clip_d1 = -(dot(Clip1.xyz, p) - Clip1.w);
    float clip_d2 = -(dot(Clip2.xyz, p) - Clip2.w);

    // normal computation
    vec3  n = gradCylinder(p);
    float s = clip_d1 / (clip_d1 + clip_d2);


    if (normalInterpolation > 0.0)
    {
        vec3  t   = normalize((1.0 - s) * Clip1.xyz + s * -Clip2.xyz);
        vec4  r_q = qRot(normalize(cross(n, vec3(0.0, 1.0, 0.0))), -angle(t, n) - 1.570796326);

        n = -rotatePoint(n, r_q);
    }
    n = -n;

    // color computation

    vec4 color = vec4(1., 0., 0., 1.);
    if (Colors.z < 0.5) color = getColor(Colors.xy, s);

    // back transformation, depth computation and shading
    p = rotatePointInv(p, Rotation);
    n = rotatePointInv(n, Rotation);
    p = p + m;

    color = illuminate(color, p, n);

    vec4 np = gl_ProjectionMatrix * vec4(p, 1.0);

    gl_FragDepth = ((np.z / np.w) + 1.0) * 0.5;
    gl_FragColor = color;
}
