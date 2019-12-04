/**
    Raycasting of a cylinder.
    ========================

    This shader raycasts a cylinder for orthographic projection
    and cuts it with a slice.

    input: cylinder start point = gl_Vertex.xyz
           cylinder end point   = gl_MultiTexCoord0.xyz
           color                = gl_Color;
           radius               = uniform float "radius"
           slice                = uniform vec4 "slice"
           slice height         = uniform float "sliceHeight"


    Vertex Shader:

    Applies the modelview matrix to the end points of the
    cylinder. 

    
    Geometry Shader:

    Computes a thight square for the cylinder and the
    ray start and direction for raycasting.


    Fragment Shader:

    Computes the intersection points of the ray and the
    cylinder. Compares the points with the slice and 
    computes the intersection with the slice if necessary.

    author: Norbert Lindow
*/


varying vec4 cylinder;
varying vec4 color;
varying vec4 rayStart;
varying vec4 rotation;

uniform vec4  slice;
uniform float sliceHeight;



/*
    Quaternion multiplikation.
*/
vec4 qMul(vec4 q1, vec4 q2)
{
    return vec4((q1.w * q2.xyz) + (q2.w * q1.xyz) + cross(q1.xyz, q2.xyz), q1.w * q2.w - dot(q1.xyz, q2.xyz));
}




/*
    Point rotation with quaternion. Axe must have unit length.
*/
vec3 rotatePoint (vec3 point, vec3 axis, float angle)
{
    vec4 q  = vec4(sin(angle * 0.5) * axis, cos(angle * 0.5));

    vec4 qC = vec4(-1.0 * q.xyz, q.w);
    vec4 x  = vec4(point, 0.0);
    vec4 res = qMul(qMul(q, x), qC);

    return res.xyz;
}




/*
    Helper function for isecCylinder.
*/
float dotC(vec3 v1, vec3 v2)
{
    return v1.x * v2.x + v1.z * v2.z;
}




/* 
  Returns the intersection points of a ray and a cylinder.
  The ray starts at "rs" and has direction "rd". The midpoint
  of the cylinder is at zero position and the radius is "r".
  The cylinder axis is equal to the Y-axis.
*/
void isecCylinder(vec3 rs, vec3 rd, float r, out vec3 isec_first, out vec3 isec_second)
{
    float w = 1.0 / dotC(rd, rd);
    float p = 2.0 * dotC(rs, rd) * w;
    float q = (dotC(rs, rs) - r * r) * w;

    w = p * p * 0.25 - q;

    if (w < 0.0) discard;

    w = sqrt(w);
    p = -0.5 * p;
    
    isec_first  = rs + (p + w) * rd;
    isec_second = rs + (p - w) * rd;
}




/*
   Returns gradient of a cylinder in zero position and y-axis direction.
*/
vec3 gradCylinder(vec3 p)
{
    return normalize(vec3(-2.0 * p.x, 0.0, -2.0 * p.z));
}




/*
    Returns the intersection point of a ray and a plane.
*/
vec3 isecPlane(vec4 plane, vec3 ray_start, vec3 ray_direction)
{
    float t = (plane.w - dot(ray_start, plane.xyz)) / dot(ray_direction, plane.xyz);

    return ray_start + t * ray_direction;
}




/*
    The method checks if a point "point" is inside the cylinder segment
    and inside the clip, given by a plane "clip_plane" and a thickness
    "clip_thickness". Note, that the axis of the cylinder segment is
    equal to the Y-axis and that the height is bounded bounded by
    [-cylinder_height, cylinder_height].
*/
bool isValid(vec3 point, float cylinder_height, vec4 clip_plane, float clip_thickness)
{
    if (abs(point.y) > cylinder_height) return false;

    float point_dist = dot(point, clip_plane.xyz) - clip_plane.w;

    return (point_dist <=  0.0 &&
            point_dist >= -clip_thickness);
}




/*
    Main.
*/
void main (void)
{
    float r     = cylinder.w;
    float h     = rayStart.w;
    vec3  m     = cylinder.xyz;
    vec3  rd    = rotatePoint(vec3(0.0, 0.0, 1.0), rotation.xyz, rotation.w);
    vec3  rs    = rayStart.xyz;

    /*
        transform center plane of slice to modelviewspace
        and move plane to one side of the slice.
    */

    float clip_thickness = 2.0 * sliceHeight;
    vec3  clip_normal    = normalize(vec3(gl_ModelViewMatrix * vec4(slice.xyz, 0.0)));
    vec3  clip_pos       = vec3(gl_ModelViewMatrix * vec4(slice.xyz * slice.w, 1.0)) - m;
          clip_pos       = clip_pos + abs(sliceHeight) * clip_normal;

    // transform this plane into the space of the cylinder

    clip_pos    = rotatePoint(clip_pos, rotation.xyz, rotation.w);
    clip_normal = rotatePoint(clip_normal, rotation.xyz, rotation.w);

    // transform back to clipping plane representation

    vec4 clip = vec4(clip_normal, dot(clip_pos, clip_normal));

    // compute intersection points with cylinder

    vec3 p_first;
    vec3 p_second;

    isecCylinder(rs, rd, r, p_first, p_second);

    vec3 p  = p_first;
    vec3 n  = gradCylinder(p);

    /*
        if first point is not valid, 
        try intersection point with plane
        and afterwards try second point
    */
    if (!isValid(p, h, clip, clip_thickness))
    {
        p  = isecPlane(clip, rs, rd);
        n  = -clip.xyz;

        if (p.x * p.x + p.z * p.z > r * r || abs(p.y) > h)
        {
            p  = p_second;
            n  = gradCylinder(p);

            if (!isValid(p, h, clip, clip_thickness)) discard;
        }
    }

    // back transformation, depth computation
    p = rotatePoint(p, rotation.xyz, -rotation.w);
    p = p + m;

    vec4 np = gl_ProjectionMatrix * vec4(p, 1.0);

    gl_FragDepth = ((np.z / np.w) + 1.0) * 0.5; 
    gl_FragColor = vec4(/*abs(dot(n, rd)) **/ color);

}