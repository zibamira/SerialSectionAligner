

#version 120
#extension GL_EXT_geometry_shader4 : enable



varying in vec3  colorValues[];
varying in vec4  Segment[];
varying in vec3  SegmentAxis[];
varying in vec3  SegmentClip1[];
varying in vec3  SegmentClip2[];

varying out vec3  rayDirection;
varying out vec4  segment;
varying out vec3  segmentAxis;
varying out vec3  segmentClip1;
varying out vec3  segmentClip2;
varying out vec3  segmentPosition;


varying out vec3  colors;
varying out float mScale;




/*
  Computation of a tight square for a cylinder.
*/


void main(void)
{
    // compute cam position (naturally in zero but not in Studio)
    vec3 c = vec3(0.0, 0.0, 0.0);

    if (gl_ProjectionMatrix[3][3] != 0.0)
    {
        vec4  c_p  = gl_ProjectionMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
              c    = (1.0 / c_p.w) * c_p.xyz;
    }


    vec3  m    = gl_PositionIn[0].xyz - c;
    float r    = gl_PositionIn[0].w;
    vec3  axis = SegmentAxis[0];

    vec3 n_y  = normalize(cross(m, axis));
    vec3 n_z  = normalize(cross(axis, n_y));

    // compute sphere height
    float d2  = min(dot(m + axis, m + axis) , dot(m - axis, m - axis));
    float d   = sqrt(d2);
    float r2  = r * r;
    float h   = sqrt(r2 + ((r2 * r2) / (d2 - r2)));


    // compute left vertex and the height
    vec3 v_1 = m - axis;
    vec3 vf  = v_1 - r * n_z;
    vec3 vb  = v_1 + r * n_z;

    float s     = 0;
    float s_1ny = 1.0;
    float s_2ny = 1.0;

    if (dot(n_y, cross(vf, vb)) > 0.0)
    {
        v_1 = vf;
        
        if (dot(cross(v_1, v_1 + 2.0 * axis), n_y) < 0.0)
        {
            v_1 = v_1 + 2.0 * axis;
        }
    }
    else
    {
        v_1 = vb;
    }

    s = (dot(v_1, v_1) - d2 - r2) / (2.0 * d);

    s_1ny = (h * (d + s)) / d;


    // compute right vertex and height

    vec3 v_2  = m + axis;
         vf   = v_2 - r * n_z;
         vb   = v_2 + r * n_z;

    if (dot(n_y, cross(vf, vb)) < 0.0)
    {
        v_2 = vf;

        if (dot(cross(v_2, v_2 - 2.0 * axis), n_y) > 0.0)
        {
            v_2 = v_2 - 2.0 * axis;
        }
    }
    else
    {
        v_2 = vb;
    }

    s = (dot(v_2, v_2) - d2 - r2) / (2.0 * d);

    s_2ny = (h * (d + s)) / d;


    // build polygon

    rayDirection    = v_1 - s_1ny * n_y;
    segment         = Segment[0];
    segmentAxis     = axis;
    segmentClip1    = SegmentClip1[0];
    segmentClip2    = SegmentClip2[0];
    segmentPosition = m;

    colors          = colorValues[0];

    gl_Position     = gl_ProjectionMatrix * vec4(c + v_1 - s_1ny * n_y, 1.0);

    EmitVertex();

    rayDirection    = v_1 + s_1ny * n_y;
    segment         = Segment[0];
    segmentAxis     = axis;
    segmentClip1    = SegmentClip1[0];
    segmentClip2    = SegmentClip2[0];
    segmentPosition = m;

    colors          = colorValues[0];

    gl_Position     = gl_ProjectionMatrix * vec4(c + v_1 + s_1ny * n_y, 1.0);

    EmitVertex();

    rayDirection    = v_2 - s_2ny * n_y;
    segment         = Segment[0];
    segmentAxis     = axis;
    segmentClip1    = SegmentClip1[0];
    segmentClip2    = SegmentClip2[0];
    segmentPosition = m;

    colors          = colorValues[0];

    gl_Position     = gl_ProjectionMatrix * vec4(c + v_2 - s_2ny * n_y, 1.0);

    EmitVertex();

    rayDirection    = v_2 + s_2ny * n_y;
    segment         = Segment[0];
    segmentAxis     = axis;
    segmentClip1    = SegmentClip1[0];
    segmentClip2    = SegmentClip2[0];
    segmentPosition = m;

    colors          = colorValues[0];

    gl_Position     = gl_ProjectionMatrix * vec4(c + v_2 + s_2ny * n_y, 1.0);

    EmitVertex();

    EndPrimitive();
}
