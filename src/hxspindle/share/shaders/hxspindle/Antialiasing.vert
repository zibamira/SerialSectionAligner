#version 420 compatibility
#extension GL_EXT_gpu_shader4 : enable

/**
    Fullscreen rendering of a grey scale texture
    ============================================


    This shader expects a default orthogonal projection with
    a single rendered quad with range [-1..1][-1..1]. The quad
    will be stretched in this shader to a full screen quad.

    author: Norbert Lindow
*/


// per vertex output

out vec2 pos; // screen position in range of [0..1][0..1]




void main (void)
{
    pos = gl_MultiTexCoord0.xy;

    gl_Position = vec4(gl_Vertex.xy, 0.0, 1.0);
}
