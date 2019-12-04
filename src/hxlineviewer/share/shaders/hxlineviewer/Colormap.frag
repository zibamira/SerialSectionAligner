
/**
    Holds the minimum and maximum values of the colormap
    in the X and Y component.
*/
uniform vec2 colormapRange;

/**
    Holds the number of colors.
*/
uniform int colormapSize;

/**
    The colormap itself as 1D texture.
*/
uniform sampler1D colormap;




/**
    Computes the correct color value for the colormap
    texture fetch.
*/
float getColorValue(float value)
{
    float v = (value - colormapRange.x) / (colormapRange.y - colormapRange.x);  // project on [0, 1]

    if (v < 0.0) return 1.0 / (float(colormapSize) * 2.0);
    if (v > 1.0) return 1.0 - (1.0 / (float(colormapSize) * 2.0));

    float w = 1.0 - (3.0 / float(colormapSize));
    float a = (1.5 / float(colormapSize));

    v = w * v + a;

    return v;
}




/**
    Computes the correct color for a value of a colormap
*/
vec4 getColor(float value, float selected)
{
    if (selected > 0.0)
        return vec4(1.0, 0.0, 0.0, 1.0);
    return texture1D(colormap, getColorValue(value));
}




/**
    Linear interpolation of two colors.
*/
vec4 getColor(vec2 values, float s)
{
    vec4 colStart = texture1D(colormap, getColorValue(values.x));
    vec4 colEnd   = texture1D(colormap, getColorValue(values.y));

    return (1.0 - s) * colStart + s * colEnd;
}