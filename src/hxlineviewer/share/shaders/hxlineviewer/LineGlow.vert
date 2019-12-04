
uniform float radiusScale;
uniform float scale;

varying vec2 colorValues;
varying vec4 segStart;
varying vec4 segEnd;
varying vec3 segTangStart;
varying vec3 segTangEnd; 



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

    colorValues     = gl_MultiTexCoord3.xy;
    segStart        = p1;
    segEnd          = p2;
    segTangStart    = p2.xyz - p0.xyz;
    segTangEnd      = p3.xyz - p1.xyz;

    gl_Position = gl_ProjectionMatrix * vec4(0.5 * (p1.xyz + p2.xyz), 1.0);

}