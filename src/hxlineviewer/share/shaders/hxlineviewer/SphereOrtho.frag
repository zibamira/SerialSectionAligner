#include "Colormap.frag"
#include "Light.frag"

// input per fragment

varying vec2  Color;
varying vec3  RayStart;
varying vec4  Sphere;

// global input

uniform int numClipPlane;


bool clipPoint(vec3 p)
{
    vec3 tp = p + Sphere.xyz;

    for (int i = 0; i < numClipPlane; ++i)
    {
        if (dot(vec4(tp, 1.0), gl_ClipPlane[i]) < 0.0) return true;
    }

    return false;
}




void main (void)
{
    vec3  rd = vec3(0.0, 0.0, 1.0);
    vec3  rs = RayStart;
    vec3  m  = Sphere.xyz;
    float r  = Sphere.w;

    float p = 2.0 * dot(rd, rs);
    float q = dot(rs, rs) - r * r;
    float w = p * p * 0.25 - q;

    if (w < 0.0) discard;

    float t     = (-p * 0.5) + sqrt(w);
    vec3  isec  = rs + t * rd;

    if (clipPoint(isec))
    {
        t    = (-p * 0.5) - sqrt(w);
        isec = rs + t * rd;

        if (clipPoint(isec)) discard;
    }

    vec3  n     = normalize(isec);
    vec4  np    = gl_ProjectionMatrix * vec4(m + isec, 1.0);
    vec4  color = getColor(Color.x, Color.y);

    color   = illuminate(color, m + isec, n);
    color.a = 1.0;

    gl_FragDepth = ((np.z / np.w) + 1.0) * 0.5;
    gl_FragColor = color;
}
