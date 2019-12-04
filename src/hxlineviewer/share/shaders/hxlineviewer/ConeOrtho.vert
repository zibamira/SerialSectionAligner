
#include "Geometry.geom"

varying vec3   colorValues;

varying vec4   Segment;
varying vec3   SegmentAxis;
varying vec3   SegmentClip1;
varying vec3   SegmentClip2;



uniform float  radiusScale;
uniform float  scale;




float eps = 0.0001;




void main (void)
{
    // read line segment
    vec4  p0 = gl_Vertex;
    vec4  p1 = gl_MultiTexCoord0;
    vec4  p2 = gl_MultiTexCoord1;
    vec4  p3 = gl_MultiTexCoord2;

    p0.w = radiusScale * p0.w * scale;
    p1.w = radiusScale * p1.w * scale;
    p2.w = radiusScale * p2.w * scale;
    p3.w = radiusScale * p3.w * scale;

    p0.xyz = vec3(gl_ModelViewMatrix * vec4(p0.xyz, 1.0));
    p1.xyz = vec3(gl_ModelViewMatrix * vec4(p1.xyz, 1.0));
    p2.xyz = vec3(gl_ModelViewMatrix * vec4(p2.xyz, 1.0));
    p3.xyz = vec3(gl_ModelViewMatrix * vec4(p3.xyz, 1.0));


    // segment clip computation

    vec3  t0 = p0.xyz - p1.xyz;
    vec3  t1 = p3.xyz - p2.xyz;

    float a0 = 0.0;
    float a1 = 0.0;

    float alpha = angle(t0, p2.xyz - p1.xyz);
    float beta  = angle(t1, p1.xyz - p2.xyz);

    if (alpha < 3.14)
    {
        alpha = 0.5 * (3.14159265 - alpha);
        a0    = p1.w * tan(alpha);

        vec3 axis = normalize(cross(t0, p2.xyz - p1.xyz));

        t0 = rotatePoint(p1.xyz - p2.xyz, axis, alpha);
    }

    if (beta < 3.14)
    {
        beta = 0.5 * (3.14159265 - beta);
        a1   = p2.w * tan(beta);

        vec3 axis = normalize(cross(t1, p1.xyz - p2.xyz));

        t1 = rotatePoint(p2.xyz - p1.xyz, axis, beta);
    }

    // cylinder 
    vec3  m          = 0.5 * (p1.xyz + p2.xyz);
    vec3  axis       = 0.5 * (p2.xyz - p1.xyz);
    float axisLength = length(axis);

    axis = axis + max(a0, a1) * normalize(axis);

    float mr = max(p1.w, p2.w);

    Segment       = vec4(axisLength, axisLength, p1.w, p2.w);
    SegmentAxis   = axis;
    SegmentClip1  = normalize(t0);
    SegmentClip2  = normalize(t1);

    gl_Position   = vec4(m, mr);

    colorValues = gl_MultiTexCoord3.xy;

}
