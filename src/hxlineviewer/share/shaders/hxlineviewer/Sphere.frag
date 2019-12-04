#include "Geometry.geom"
#include "Light.frag"



varying vec4 colorValue;

uniform int numClipPlane;



bool clipPoint(vec3 p)
{
    for (int i = 0; i < numClipPlane; ++i)
    {
        if (dot(vec4(p, 1.0), gl_ClipPlane[i]) < 0.0) return true;
    }

    return false;
}


void main (void)
{

    vec3 c = cameraPositionInViewSpace();

    vec3  rd = normalize(gl_TexCoord[1].xyz);
    vec3  m  = gl_TexCoord[0].xyz; //sphere.xyz;
    float r  = gl_TexCoord[0].w;   //sphere.w;

    float p = -2.0 * dot(rd, m);
    float q = dot(m, m) - r * r;
    float w = p * p * 0.25 - q;

    if (w < 0.0) discard;

    float t = (-p * 0.5) - sqrt(w);

    if (clipPoint(c + t * rd))
    {
        t    = (-p * 0.5) + sqrt(w);

        if (clipPoint(c + t * rd)) discard;
    }

    vec3  n = normalize((t * rd) - m);

    vec4 color = illuminate(colorValue, c + t * rd, n);

    vec4 np = gl_ProjectionMatrix * vec4(c + t * rd, 1.0);

    gl_FragDepth = ((np.z / np.w) + 1.0) * 0.5;
    gl_FragColor = color;
}