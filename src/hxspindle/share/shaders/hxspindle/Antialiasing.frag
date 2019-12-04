#version 420 compatibility
#extension GL_EXT_gpu_shader4 : enable

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

in vec2            pos; // screen position in range of [0..1][0..1]

// global input

uniform sampler2D  image; // texture with grey values
uniform vec2       window; // window size in pixel coordinates
uniform float      width = 1.0;


const int maxSteps = 10;


ivec2 follow(ivec2 dir, ivec2 side, ivec2 p)
{
    for (int i = 1; i < maxSteps; ++i)
    {
        float w1 = texelFetch2D(image, p + i * dir, 0).w;
        float w2 = texelFetch2D(image, p + i * dir + side, 0).w;

        if (w1 != 0.0) return ivec2(i, 0);
        if (w2 == 0.0) return ivec2(i, 1);
    }

    return ivec2(maxSteps, 2);
}




ivec2 follow2(ivec2 dir, ivec2 side, ivec2 p)
{
    for (int i = 1; i < maxSteps; ++i)
    {
        float w1 = texelFetch2D(image, p + i * dir, 0).w;
        float w2 = texelFetch2D(image, p + i * dir + side, 0).w;

        if (w1 == 0.0) return ivec2(i, 1);
        if (w2 != 0.0) return ivec2(i, 0);
    }

    return ivec2(maxSteps, 2);
}




vec4 blend(ivec2 e1, ivec2 e2)
{
    if (e1.y == 2 && e2.y == 2) return vec4(0.0, 0.0, 1.0, 0.5);

    if (e1.y == 1 && e2.y == 1) return vec4(1.0, 0.0, 1.0, float(min(e1.x, e2.x)) / float(e1.x + e2.x));
    if (e1.y == 0 && e2.y == 0) return vec4(1.0, 0.0, 1.0, 1.0 - float(min(e1.x, e2.x)) / float(e1.x + e2.x));

    if (e1.y == 0 && e2.y == 1) return vec4(1.0, 0.0, 0.0, float(e2.x) / float(e1.x + e2.x));
    if (e1.y == 2 && e2.y == 1) return vec4(1.0, 0.0, 0.0, float(e2.x) / float(e1.x + e2.x));
    if (e1.y == 0 && e2.y == 2) return vec4(1.0, 0.0, 0.0, float(e2.x) / float(e1.x + e2.x));
    if (e1.y == 1 && e2.y == 0) return vec4(0.0, 1.0, 0.0, float(e1.x) / float(e1.x + e2.x));
    if (e1.y == 1 && e2.y == 2) return vec4(0.0, 1.0, 0.0, float(e1.x) / float(e1.x + e2.x));
    if (e1.y == 2 && e2.y == 0) return vec4(0.0, 1.0, 0.0, float(e1.x) / float(e1.x + e2.x));


    return vec4(0.0, 0.0, 0.0, 1.0);
}




void main (void)
{

    // current position in pixel coordinates
    ivec2 p = ivec2(pos * window + vec2(0.1));

    vec4 c = texelFetch2D(image, p, 0);

    vec4 l = texelFetch2D(image, p - ivec2(1, 0), 0);
    vec4 r = texelFetch2D(image, p + ivec2(1, 0), 0);
    vec4 u = texelFetch2D(image, p + ivec2(0, 1), 0);
    vec4 d = texelFetch2D(image, p - ivec2(0, 1), 0);


    if (c.w == 0)
    {
        int   w  = 0;
        ivec2 e1 = ivec2(0, 0);
        ivec2 e2 = ivec2(0, 0);
        vec4  n  = ivec4(0.0, 0.0, 0.0, 0.0);

        if (r.w != 0.0)
        {
            ivec2 end1 = follow(ivec2(0,  1), ivec2(1, 0), p);
            ivec2 end2 = follow(ivec2(0, -1), ivec2(1, 0), p);

            if (end1.x + end2.x > w)
            {
                w  = end1.x + end2.x;
                e1 = end1;
                e2 = end2;
                n  = r;
            }
        }
        if (l.w != 0.0)
        {
            ivec2 end1 = follow(ivec2(0,  1), ivec2(-1, 0), p);
            ivec2 end2 = follow(ivec2(0, -1), ivec2(-1, 0), p);

            if (end1.x + end2.x > w)
            {
                w  = end1.x + end2.x;
                e1 = end1;
                e2 = end2;
                n  = l;
            }
        }
        if (u.w != 0.0)
        {
            ivec2 end1 = follow(ivec2( 1, 0), ivec2(0, 1), p);
            ivec2 end2 = follow(ivec2(-1, 0), ivec2(0, 1), p);

            if (end1.x + end2.x > w)
            {
                w  = end1.x + end2.x;
                e1 = end1;
                e2 = end2;
                n  = u;
            }
        }
        if (d.w != 0.0)
        {
            ivec2 end1 = follow(ivec2( 1, 0), ivec2(0, -1), p);
            ivec2 end2 = follow(ivec2(-1, 0), ivec2(0, -1), p);

            if (end1.x + end2.x > w)
            {
                w  = end1.x + end2.x;
                e1 = end1;
                e2 = end2;
                n  = d;
            }
        }

        if (w > 0)
        {
            vec4 b = blend(e1, e2);

            float s = (width - 10000.0) / 30000.0;

            s = clamp(s, 0.0, 1.0);
            s = sqrt(s);


            //c = vec4(n.xyz, b.w);
            c = s * vec4(n.xyz, b.w) + (1.0 - s) * vec4(vec3(0.0), b.w);
            //c = b;
        }
    }

    if (width < 40000.0)
    {
        float s = (width - 10000.0) / 30000.0;

        s = clamp(s, 0.0, 1.0);
        s = sqrt(s);

        if (c.w > 0)
        {
            int   w  = 0;
            ivec2 e1 = ivec2(0, 0);
            ivec2 e2 = ivec2(0, 0);
            vec4  n  = ivec4(0.0, 0.0, 0.0, 0.0);

            if (r.w == 0.0)
            {
                ivec2 end1 = follow2(ivec2(0,  1), ivec2(1, 0), p);
                ivec2 end2 = follow2(ivec2(0, -1), ivec2(1, 0), p);

                if (end1.x + end2.x > w)
                {
                    w  = end1.x + end2.x;
                    e1 = end1;
                    e2 = end2;
                    n  = r;
                }
            }
            if (l.w == 0.0)
            {
                ivec2 end1 = follow2(ivec2(0,  1), ivec2(-1, 0), p);
                ivec2 end2 = follow2(ivec2(0, -1), ivec2(-1, 0), p);

                if (end1.x + end2.x > w)
                {
                    w  = end1.x + end2.x;
                    e1 = end1;
                    e2 = end2;
                    n  = l;
                }
            }
            if (u.w == 0.0)
            {
                ivec2 end1 = follow2(ivec2( 1, 0), ivec2(0, 1), p);
                ivec2 end2 = follow2(ivec2(-1, 0), ivec2(0, 1), p);

                if (end1.x + end2.x > w)
                {
                    w  = end1.x + end2.x;
                    e1 = end1;
                    e2 = end2;
                    n  = u;
                }
            }
            if (d.w == 0.0)
            {
                ivec2 end1 = follow2(ivec2( 1, 0), ivec2(0, -1), p);
                ivec2 end2 = follow2(ivec2(-1, 0), ivec2(0, -1), p);

                if (end1.x + end2.x > w)
                {
                    w  = end1.x + end2.x;
                    e1 = end1;
                    e2 = end2;
                    n  = d;
                }
            }

            if (w > 0)
            {
                vec4 b = blend(e1, e2);

                //c = vec4(n.xyz, b.w);
                c.xyz = s * c.xyz + (1.0 - s) * b.w * c.xyz;
                //c = b;
            }
        }
    }

    gl_FragColor = c;
}
