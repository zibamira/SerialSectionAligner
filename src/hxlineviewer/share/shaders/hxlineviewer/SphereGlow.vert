#include "Colormap.frag"

uniform float radiusScale;
uniform float scale;

varying vec4 color;


void main (void)
{
    //sphere in modelview space 
    vec3  m = vec3(gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0));
    float r = gl_Vertex.w * radiusScale * scale;

    gl_Position    = vec4(m, r);
    color          = getColor(gl_MultiTexCoord0.x, gl_MultiTexCoord0.y);
}

