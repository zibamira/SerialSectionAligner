
#include "Colormap.frag"
#include "Geometry.geom"
#include "Light.frag"


// per fragment input

varying vec3  colors;
varying vec3  rayDirection;
varying vec4  segment;
varying vec3  segmentAxis;
varying vec3  segmentClip1;
varying vec3  segmentClip2;
varying vec3  segmentPosition;
varying float segmentScale;

// global input

uniform float radiusScale;
uniform float normalInterpolation;


float eps = 0.0;



/**
    Computes iteration start
*/
vec3 cmpStart(vec4 plane1, vec4 plane2, vec3 rd, vec3 m)
{
    if (dot(-m, plane1.xyz) - plane1.w <= 0.0 &&
        dot(-m, plane2.xyz) - plane2.w <= 0.0)
    {
        return -m;
    }
    else
    {
        vec3  res = -m;

        float t1 = (plane1.w + dot(m, plane1.xyz)) / dot(rd, plane1.xyz);
        float t2 = (plane2.w + dot(m, plane2.xyz)) / dot(rd, plane2.xyz);

        if (t1 < t2)
        {
            res = t1 * rd - m;

            if (dot(res, plane2.xyz) -  plane2.w > 0.0) res = t2 * rd - m;
        }
        else
        {
            res = t2 * rd - m;

            if (dot(res, plane1.xyz) -  plane1.w > 0.0) res = t1 * rd - m;
        }

        return res;
    }
}




void main (void)
{
    // compute cam position (naturally in zero but not in Studio)
    vec3 c = vec3(0.0, 0.0, 0.0);

    if (gl_ProjectionMatrix[3][3] != 0.0)
    {
        vec4  c_p  = gl_ProjectionMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
              c    = (1.0 / c_p.w) * c_p.xyz;
    }

    // reading data
    vec3  rd        = normalize(rayDirection);
    vec3  m         = segmentPosition;
    float scale     = 1.0 / min(segment.x + segment.y, max(0.00001, min(segment.z, segment.w)));

    vec3  seg_axis  = normalize(segmentAxis);
    vec3  seg_start = scale * -segment.x * seg_axis;
    vec3  seg_end   = scale *  segment.y * seg_axis;

    vec4  clip1     = vec4(segmentClip1.xyz, dot(seg_start - eps * seg_axis, segmentClip1.xyz));
    vec4  clip2     = vec4(segmentClip2.xyz, dot(seg_end   + eps * seg_axis, segmentClip2.xyz));

    float rad_start = scale * segment.z;
    float rad_end   = scale * segment.w;

    m = scale * m;

    vec3 p  = cmpStart(clip1, clip2, rd, m);
    vec3 n  = vec3(1.0, 0.0, 0.0);

    float pg = 999999999.0;
    float go = 0.0;
    float d1 = 0.0;
    float d2 = 0.0;

    // sphere tracing

    for (int i = 0; i < 256; ++i)
    {
        vec3 p1 = cross(seg_axis, p);
        vec3 r1 = seg_start + rad_start * -normalize(cross(p1, clip1.xyz));
        vec3 r2 = seg_end   + rad_end   *  normalize(cross(p1, clip2.xyz));

        n = normalize(cross(p1, r1 - r2));

        float d  = dot(r1, n);

        go = -(dot(n, p) - d);

        if (pg < go && pg > 0.5) break;

        pg = go;

        p = p + go * rd;

        if (abs(go) < 0.01) break;
    }

    d1 = dot(clip1.xyz, p) - clip1.w;
    d2 = dot(clip2.xyz, p) - clip2.w;

    if (go > 0.01 || d1 > 0.001 || d2 > 0.001) discard;


    // normal interpolation

    float s = d1 / (d1 + d2);

    if (normalInterpolation > 0.0 )
    {
        vec3 t = normalize((1.0 - s) * clip1.xyz + s * -clip2.xyz);
        n      = -rotatePoint(n, normalize(cross(n, seg_axis)), -angle(t, n) - 1.570796326);
    }
    n = -n;

    // color computation
    vec4 color = vec4(1., 0., 0., 1.);
    if (colors.z < 0.5) color = getColor(colors, s);

    // back transformation, depth computation and shading
    p = (p + m) * (1.0 / scale);

    color = illuminate(color, p + c, n);

    vec4 np = gl_ProjectionMatrix * vec4(c + p, 1.0);

    gl_FragDepth = ((np.z / np.w) + 1.0) * 0.5; 
    gl_FragColor = color;
}
