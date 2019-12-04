varying vec4      colorValue;
varying vec2      position;

uniform float     glow;
uniform float     glowFunc;



void main (void)
{

    // compute cam position
    vec3 c = vec3(0.0, 0.0, 0.0);

    if (gl_ProjectionMatrix[3][3] != 0.0)
    {
        vec4 c_p  = gl_ProjectionMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
             c    = (1.0 / c_p.w) * c_p.xyz;
    }

    float l = length(position);

    if (l > 1.0) discard;

    float alpha = 1.0 - l;
          alpha = -pow(1.0 - pow(alpha, glow), 1.0 / glow) + 1.0;

    vec4 color = colorValue;

    if (glowFunc > 0.5) color.xyz = vec3(1.0, 1.0, 1.0) - color.xyz;

    color.a = alpha;

    gl_FragColor = color;
}