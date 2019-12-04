/**
    Fullscreen rendering of a grey scale texture
    ============================================

    The full screen quad will be colored according
    to the texture "densities". The texture can be
    larger than the screen. Therefore the user must
    specify the screen and the texture size in "view".
    Only the part of the texture within the screen
    will be rendered. Example: screen resolution is
    950x430 and texture resolution is 1024x512 then
    "view" must be set to (950.0, 430.0, 1024.0, 512.0).
    With "colorMap", the grey range can be specified.
    Therefore, the user specifies the original range,
    for example 0..255 in the x,y coordinates of
    "colorMap" and the local range, for example, 50..128
    in the z,w coordinates. This results into a
    projection of the the grey value from [0..1] to
    [0..255]. Then, the relation of this value to the
    range [50..128] is computed. So the grey value 0.1
    will be projected to 25.5 and its relation to
    [50..128] is -0.314. Finally, this value will be
    clamped to [0..1].


    author: Norbert Lindow
*/

// per fragment input

varying vec2       pos; // screen position in range of [0..1][0..1]

// global input

uniform vec4       colorMap;  // colormap (see shader description)
uniform sampler2D  densities; // texture with grey values
uniform vec4       view;      // view (see shader description)




void main (void)
{
    vec2  p = pos * (view.xy / view.zw);
    float d = texture2D(densities, p).r;

    d = d * (colorMap.y - colorMap.x) + colorMap.x;
    d = (d - colorMap.z) / (colorMap.w - colorMap.z);
    d = clamp(d, 0.0, 1.0);

    gl_FragColor = vec4(d, d, d, 1.0);
}
