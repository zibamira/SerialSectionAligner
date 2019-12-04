#include "Colormap.frag"

varying vec2        colors;
varying vec4        SegStart;
varying vec4        SegEnd;
varying vec2        SegDist;

uniform float       glow;
uniform float       glowFunc;
uniform float       normalInterpolation;


void main (void)
{
    // compute cam position (naturally in zero but not in Studio)
    vec3 c = vec3(0.0, 0.0, 0.0);

    if (gl_ProjectionMatrix[3][3] != 0.0)
    {
        vec4  c_p  = gl_ProjectionMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
              c    = (1.0 / c_p.w) * c_p.xyz;
    }

    // color computation

    vec4 color = getColor(colors, SegDist.y);

    if (glowFunc > 0.5) color.xyz = vec3(1.0, 1.0, 1.0) - color.xyz;

    float alpha = 1.0 - abs(SegDist.x);
          alpha = -pow(1.0 - pow(alpha, glow), 1.0 / glow) + 1.0;

    color.a = alpha;

    gl_FragColor = color;
}