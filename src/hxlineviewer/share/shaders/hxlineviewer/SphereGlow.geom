/* !!GLSL */


#version 120
#extension GL_EXT_geometry_shader4 : enable


varying in  vec4 color[];

varying out vec4 colorValue;
varying out vec2  position;




void main(void)
{
    // compute cam position (naturally in zero but not in Studio)
    vec3 c = vec3(0.0, 0.0, 0.0);

    if (gl_ProjectionMatrix[3][3] != 0.0)
    {
        vec4  c_p  = gl_ProjectionMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
              c    = (1.0 / c_p.w) * c_p.xyz;
    }

    vec3  m   = gl_PositionIn[0].xyz - c;
    float r   = gl_PositionIn[0].w;

    vec3  n1  = r * normalize(cross(m, vec3(0.4, 0.2, 0.7)));
    vec3  n2  = r * normalize(cross(m, n1));


    colorValue  = color[0];
    position    = vec2(-1.0, -1.0);
    gl_Position = gl_ProjectionMatrix * vec4(c + m - n1 - n2, 1.0);

    EmitVertex();

    colorValue  = color[0];
    position    = vec2(-1.0, 1.0);
    gl_Position = gl_ProjectionMatrix * vec4(c + m - n1 + n2, 1.0);

    EmitVertex();

    colorValue  = color[0];
    position    = vec2(1.0, -1.0);
    gl_Position = gl_ProjectionMatrix * vec4(c + m + n1 - n2, 1.0);

    EmitVertex();

    colorValue  = color[0];
    position    = vec2(1.0, 1.0);
    gl_Position = gl_ProjectionMatrix * vec4(c + m + n1 + n2, 1.0);

    EmitVertex();

    EndPrimitive();
}
