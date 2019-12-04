/**
    Ray casting of a plane within an axis aligned box
    =================================================

    This shader expects a default orthogonal projection with.
    The shader computes the 3D position of the view rays with
    a slice plane in an axis aligned box.

    author: Norbert Lindow
*/



// per fragment input

varying vec3 pos; // screen position in the range [-1..1][-1..1]


// global input

uniform vec3 boxMin; // minimal corner of axis aligned box
uniform vec3 boxMax; // maximal corner of axis aligned box
uniform vec4 slice;  // plane in Hesse normal form n = (x,y,z), d = w


void main (void)
{
    vec3 boxDif  = boxMax - boxMin;

    // ray computation (ortho)

    vec3  r_start = vec3(gl_ModelViewProjectionMatrixInverse * vec4(pos, 1.0));
    vec3  r_dir   = normalize(vec3(gl_ModelViewProjectionMatrixInverse * vec4(0.0, 0.0, -1.0, 0.0)));

    // ray plane interection

    float t       = (slice.w - dot(slice.xyz, r_start)) / dot(slice.xyz, r_dir);
    vec3  p       = r_start + t * r_dir;
    float a       = 1.0;

    // test if intersection point is outside the box

    if (p.x < boxMin.x || p.y < boxMin.y || p.z < boxMin.z ||
        p.x > boxMax.x || p.y > boxMax.y || p.z > boxMax.z) a = 0.1;

    //p = (p - boxMin) / (boxMax - boxMin);

    gl_FragColor = vec4(p, a);
}
