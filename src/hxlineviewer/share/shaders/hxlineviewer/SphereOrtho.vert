
// input global

uniform float radiusScale;
uniform float scale;

// output per vertex

varying vec2 color;



void main (void)
{
    //sphere in modelview space
    gl_Position    = vec4(vec3(gl_ModelViewMatrix * vec4(gl_Vertex.xyz, 1.0)), gl_Vertex.w * radiusScale * scale);
    color          = gl_MultiTexCoord0.xy;
}

