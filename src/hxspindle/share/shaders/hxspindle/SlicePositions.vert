/**
    Ray casting of a plane within an axis aligned box
    =================================================

    This shader expects a default orthogonal projection with
    a single rendered quad with range [-1..1][-1..1]. The quad
    will be stretched in this shader to a full screen quad.
    The ray casting will be performed in the fragment shader.

    author: Norbert Lindow
*/

// per vertex output

varying vec3 pos; // screen position in the range [-1..1][-1..1]



void main (void)
{
            pos = vec3(gl_Vertex.xy, 0.0);
    gl_Position = vec4(gl_Vertex.xy, 0.0, 1.0);
}
