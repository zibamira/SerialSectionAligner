#include "HxGLTexture1D.h"

#include <hxcore/HxMessage.h>

HxGLTexture1D::GLTexture1D::GLTexture1D()
{
    glGenTextures(1, &mTexture);
}

HxGLTexture1D::GLTexture1D::~GLTexture1D()
{
    glDeleteTextures(1, &mTexture);
}

void HxGLTexture1D::GLTexture1D::updateResource(void* data)
{
    Texture1DData* tData = (Texture1DData*) data;

    glBindTexture(GL_TEXTURE_1D, mTexture);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, tData->mMagFilter);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, tData->mMinFilter);
    glTexParameteri(GL_TEXTURE_1D,GL_TEXTURE_WRAP_S, tData->mWrap_S);

    glTexImage1D(GL_TEXTURE_1D,
                 0,
                 tData->mInternalFormat,
                 tData->mWidth,
                 tData->mBorder,
                 tData->mFormat,
                 tData->mType,
                 tData->mData);

    glBindTexture(GL_TEXTURE_1D, 0);
}






HxGLTexture1D::HxGLTexture1D(void)
{
}




HxGLTexture1D::~HxGLTexture1D(void)
{
}




void HxGLTexture1D::bind(unsigned int contextID, HxGLShader& shader, int texunit, const char* name)
{
    mTexture.update(contextID, &mData);

    GLTexture1D* texture = mTexture.getInstance(contextID);

    //glEnable(GL_TEXTURE_1D);
    glActiveTexture(GL_TEXTURE0 + texunit);
    glBindTexture(GL_TEXTURE_1D, texture->mTexture);
    shader.setTextureUnit(contextID, name, texunit);
    glActiveTexture(GL_TEXTURE0);
}




GLuint HxGLTexture1D::getTexture(unsigned int contextID, bool update)
{
    if (update) mTexture.update(contextID, &mData);

    GLTexture1D* texture = mTexture.getInstance(contextID);

    return texture->mTexture;
}




void HxGLTexture1D::setFetchMode(GLint mode)
{
    if (mode != GL_NEAREST && mode != GL_LINEAR) return;

    mData.mMagFilter = mode;
    mData.mMinFilter = mode;

    mTexture.touch();
}




void HxGLTexture1D::setFormat(GLint internalFormat, GLenum format, GLenum type)
{
    mData.mInternalFormat = internalFormat;
    mData.mFormat         = format;
    mData.mType           = type;

    mTexture.touch();
}




void HxGLTexture1D::setWrap(GLint param)
{
    mData.mWrap_S = param;

    mTexture.touch();
}




void HxGLTexture1D::setSize(GLsizei width)
{
    mData.mWidth  = std::max(1, std::min(width,  4096));

    mTexture.touch();
}




void HxGLTexture1D::setTextureData(void *data)
{
    mData.mData   = data;

    mTexture.touch();
}



void HxGLTexture1D::unbind(int texunit)
{
    glActiveTexture(GL_TEXTURE0 + texunit);
    glBindTexture(GL_TEXTURE_1D, 0);
    glActiveTexture(GL_TEXTURE0);
}


