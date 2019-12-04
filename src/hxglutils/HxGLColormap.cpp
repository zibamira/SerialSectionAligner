#include <algorithm>

#include <hxcolor/HxPortColormap.h>

#include "HxGLColormap.h"

HxGLColormap::GLColormap::GLColormap()
{
    glGenTextures(1, &mTexture);
}




HxGLColormap::GLColormap::~GLColormap()
{
    glDeleteTextures(1, &mTexture);
}




void HxGLColormap::GLColormap::updateResource(void *data)
{
    ColormapData* cData = (ColormapData*) data;

    glEnable(GL_TEXTURE_1D);

    glBindTexture(GL_TEXTURE_1D, mTexture);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB8, cData->mSize, 0, GL_RGB, GL_UNSIGNED_BYTE, cData->mColors);

    glBindTexture(GL_TEXTURE_1D, 0);

    glDisable(GL_TEXTURE_1D);
}






HxGLColormap::HxGLColormap(void)
{
    mData.mColors = 0;
    mData.mSize   = 0;
    mMinValue     = 0.f;
    mMaxValue     = 1.f;

    setSize(256);
}




HxGLColormap::~HxGLColormap(void)
{
    if (mData.mColors != 0) delete[] mData.mColors;
}




void HxGLColormap::disable()
{
    glEnable(GL_TEXTURE_1D);
    glBindTexture(GL_TEXTURE_1D, 0);
    glDisable(GL_TEXTURE_1D);
}




void HxGLColormap::enable(unsigned int contextID)
{
    mColormap.update(contextID, &mData);

    GLColormap* colormap = mColormap.getInstance(contextID);

    glEnable(GL_TEXTURE_1D);
    glBindTexture(GL_TEXTURE_1D, colormap->mTexture);
    glDisable(GL_TEXTURE_1D);
}

void HxGLColormap::fill(HxPortColormap* colmap, int size)
{
    if (size <= 4) return;

    size = std::min(mData.mSize, size);

    float start = colmap->getMinValue();
    float end   = colmap->getMaxValue();
    float step  = (end - start) / (float) (size - 3);

    start -= step;

    float r = 0.0;
    float g = 0.0;
    float b = 0.0;

    for (int i = 0; i < size; ++i)
    {
        SbColor col = colmap->getColor(start);

        col.getValue(r, g, b);

        start += step;

        mData.mColors[i * 3 + 0] = (unsigned char) (r * 255.0f);
        mData.mColors[i * 3 + 1] = (unsigned char) (g * 255.0f);
        mData.mColors[i * 3 + 2] = (unsigned char) (b * 255.0f);
    }

    mMinValue = colmap->getMinValue();
    mMaxValue = colmap->getMaxValue();

    mColormap.touch();
}




void HxGLColormap::fill(float* rgb)
{
    if (!rgb) return;

    for (int i = 0; i < mData.mSize; ++i)
    {
        mData.mColors[i * 3 + 0] = (unsigned char) (rgb[0] * 255.0f);
        mData.mColors[i * 3 + 1] = (unsigned char) (rgb[1] * 255.0f);
        mData.mColors[i * 3 + 2] = (unsigned char) (rgb[2] * 255.0f);

    }

    mColormap.touch();
}





void HxGLColormap::fill(float red, float green, float blue)
{
    for (int i = 0; i < mData.mSize; ++i)
    {
        mData.mColors[i * 3 + 0] = (unsigned char) (red   * 255.0f);
        mData.mColors[i * 3 + 1] = (unsigned char) (green * 255.0f);
        mData.mColors[i * 3 + 2] = (unsigned char) (blue  * 255.0f);
    }

    mColormap.touch();
}




void HxGLColormap::fillRandom()
{
    for (int i = 0; i < mData.mSize; ++i)
    {
        mData.mColors[i * 3 + 0] = (unsigned char) (255.0 * (float) rand() / (float) RAND_MAX);
        mData.mColors[i * 3 + 1] = (unsigned char) (255.0 * (float) rand() / (float) RAND_MAX);
        mData.mColors[i * 3 + 2] = (unsigned char) (255.0 * (float) rand() / (float) RAND_MAX);
    }

    mColormap.touch();
}




void HxGLColormap::setColor(float* rgb, int index)
{
    if   (index > mData.mSize - 1 || index < 0) return;

    mData.mColors[index * 3 + 0] = (unsigned char) (rgb[0] * 255.0);
    mData.mColors[index * 3 + 1] = (unsigned char) (rgb[1] * 255.0);
    mData.mColors[index * 3 + 2] = (unsigned char) (rgb[2] * 255.0);

    mColormap.touch();
}




void HxGLColormap::setMinMax(float min, float max)
{
    mMinValue = min;
    mMaxValue = max;
}




void HxGLColormap::setSize(int size)
{
    if (size == mData.mSize) return;

    int s = size;
    while (s % 2 == 0) s /= 2;

    if (s != 1) return;

    if (mData.mColors != 0) delete[] mData.mColors;

    mData.mSize   = size;
    mData.mColors = new unsigned char[mData.mSize * 3];

    mColormap.touch();
}




HxGLColormap& HxGLColormap::operator=(const HxGLColormap &colmap)
{
    setSize(colmap.mData.mSize);

    for (int i = 0; i < mData.mSize * 3; ++i)
    {
        mData.mColors[i] = colmap.mData.mColors[i];
    }

    mMinValue = colmap.mMinValue;
    mMaxValue = colmap.mMaxValue;

    mColormap.touch();

    return *this;
}

