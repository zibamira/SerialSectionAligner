/* !!GLSL */


#version 120
#extension GL_EXT_geometry_shader4 : enable

varying in vec2 colorValues[];
varying in vec4 segStart[];
varying in vec4 segEnd[];
varying in vec3 segTangStart[];
varying in vec3 segTangEnd[];

varying out vec2 colors;
varying out vec4 SegStart;
varying out vec4 SegEnd;
varying out vec2 SegDist;




void main(void)
{

    // compute cam position (naturally in zero but not in Studio)
    vec3 c = vec3(0.0, 0.0, 0.0);

    if (gl_ProjectionMatrix[3][3] != 0.0)
    {
        vec4  c_p  = gl_ProjectionMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
              c    = (1.0 / c_p.w) * c_p.xyz;
    }

    vec3  p1 = segStart[0].xyz - c;
    vec3  p2 = segEnd[0].xyz - c;
    float r1 = segStart[0].w;
    float r2 = segEnd[0].w;
    vec3  t1 = segTangStart[0];
    vec3  t2 = segTangEnd[0];

    vec3 n1  = normalize(cross(vec3(0.0, 0.0, -1.0), t1));
    vec3 n2  = normalize(cross(vec3(0.0, 0.0, -1.0), t2));

    // output

    colors         = colorValues[0];
    SegStart       = segStart[0];
    SegEnd         = segEnd[0];
    SegDist        = vec2(1.0, 0.0);

    gl_Position = gl_ProjectionMatrix * vec4(c + p1 + r1 * n1, 1.0);
    EmitVertex();

    colors         = colorValues[0];
    SegStart       = segStart[0];
    SegEnd         = segEnd[0];
    SegDist        = vec2(-1.0, 0.0);

    gl_Position = gl_ProjectionMatrix * vec4(c + p1 - r1 * n1, 1.0);
    EmitVertex();

    colors         = colorValues[0];
    SegStart       = segStart[0];
    SegEnd         = segEnd[0];
    SegDist        = vec2(1.0, 1.0);

    gl_Position = gl_ProjectionMatrix * vec4(c + p2 + r2 * n2, 1.0);
    EmitVertex();

    colors         = colorValues[0];
    SegStart       = segStart[0];
    SegEnd         = segEnd[0];
    SegDist        = vec2(-1.0, 1.0);

    gl_Position = gl_ProjectionMatrix * vec4(c + p2 - r2 * n2, 1.0);
    EmitVertex();

    EndPrimitive();
}