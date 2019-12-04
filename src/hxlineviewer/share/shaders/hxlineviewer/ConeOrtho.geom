#version 120
#extension GL_EXT_geometry_shader4 : enable


#include "Geometry.geom"


varying in vec3  colorValues[];
varying in vec4  Segment[];
varying in vec3  SegmentAxis[];
varying in vec3  SegmentClip1[];
varying in vec3  SegmentClip2[];


varying out vec3  colors;
varying out vec3  rayStart;
varying out vec4  segment;
varying out vec3  segmentAxis;
varying out vec3  segmentClip1;
varying out vec3  segmentClip2;
varying out vec3  segmentPosition;




/*
  Computation of a tight square for a cylinder.
*/


void main(void)
{
    vec3  m     = gl_PositionIn[0].xyz;
    float r     = gl_PositionIn[0].w;
    vec3  axis  = SegmentAxis[0];
    float h     = length(axis);

    vec3 z_axis = vec3(0.0, 0.0, 1.0);
    vec3 n_y    = normalize(cross(z_axis, axis));
    vec3 n_z    = normalize(cross(axis, n_y));

    float s     = -r;

    if (angle(axis, z_axis) < 1.5707963) s = -s;

    // build polygon

    rayStart        = -axis + s * n_z + r * n_y;
    segment         = Segment[0];
    segmentAxis     = axis;
    segmentClip1    = SegmentClip1[0];
    segmentClip2    = SegmentClip2[0];
    segmentPosition = m;

    colors          = colorValues[0];

    gl_Position     = gl_ProjectionMatrix * vec4(m - axis + s * n_z + r * n_y, 1.0);

    EmitVertex();

    rayStart        = -axis + s * n_z - r * n_y;
    segment         = Segment[0];
    segmentAxis     = axis;
    segmentClip1    = SegmentClip1[0];
    segmentClip2    = SegmentClip2[0];
    segmentPosition = m;

    colors          = colorValues[0];

    gl_Position     = gl_ProjectionMatrix * vec4(m - axis + s * n_z - r * n_y, 1.0);

    EmitVertex();

    rayStart        = axis - s * n_z + r * n_y;
    segment         = Segment[0];
    segmentAxis     = axis;
    segmentClip1    = SegmentClip1[0];
    segmentClip2    = SegmentClip2[0];
    segmentPosition = m;

    colors          = colorValues[0];

    gl_Position     = gl_ProjectionMatrix * vec4(m + axis - s * n_z + r * n_y, 1.0);

    EmitVertex();

    rayStart        = axis - s * n_z - r * n_y;
    segment         = Segment[0];
    segmentAxis     = axis;
    segmentClip1    = SegmentClip1[0];
    segmentClip2    = SegmentClip2[0];
    segmentPosition = m;

    colors          = colorValues[0];

    gl_Position     = gl_ProjectionMatrix * vec4(m + axis - s * n_z - r * n_y, 1.0);

    EmitVertex();

    EndPrimitive();
}
