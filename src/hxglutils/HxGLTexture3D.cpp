#include "HxGLTexture3D.h"

#include <hxcore/HxMessage.h>

void
HxGLTexture3D::GLTexture3D::updateResource(void* data)
{
    Texture3DData* tData = (Texture3DData*)data;

    glBindTexture(GL_TEXTURE_3D, mTexture);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, tData->mMagFilter);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, tData->mMinFilter);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, tData->mWrap_S);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, tData->mWrap_T);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, tData->mWrap_R);

    glTexImage3D(GL_TEXTURE_3D,
                 0,
                 tData->mInternalFormat,
                 tData->mWidth,
                 tData->mHeight,
                 tData->mDepth,
                 tData->mBorder,
                 tData->mFormat,
                 tData->mType,
                 tData->mData);

    glBindTexture(GL_TEXTURE_3D, 0);
}

HxGLTexture3D::HxGLTexture3D(void)
{
}

HxGLTexture3D::~HxGLTexture3D(void)
{
}

void
HxGLTexture3D::activate(unsigned int contextID)
{
    mTexture.update(contextID, &mData);
}

void
HxGLTexture3D::bind(unsigned int contextID, int texunit)
{
    mTexture.update(contextID, &mData);

    GLTexture3D* texture = mTexture.getInstance(contextID);

    glActiveTexture(GL_TEXTURE0 + texunit);
    glBindTexture(GL_TEXTURE_3D, texture->mTexture);
    glActiveTexture(GL_TEXTURE0);
}

GLuint
HxGLTexture3D::getTexture(unsigned int contextID, bool update)
{
    if (update)
        mTexture.update(contextID, &mData);

    GLTexture3D* texture = mTexture.getInstance(contextID);

    return texture->mTexture;
}

void
HxGLTexture3D::setFetchMode(GLint mode)
{
    if (mode != GL_NEAREST && mode != GL_LINEAR)
        return;

    mData.mMagFilter = mode;
    mData.mMinFilter = mode;

    mTexture.touch();
}

void
HxGLTexture3D::setFormat(GLint internalFormat, GLenum format, GLenum type)
{
    mData.mInternalFormat = internalFormat;
    mData.mFormat = format;
    mData.mType = type;

    mTexture.touch();
}

void
HxGLTexture3D::setSize(GLsizei width, GLsizei height, GLsizei depth)
{
    mData.mHeight = std::max(1, std::min(height, 4096));
    mData.mWidth = std::max(1, std::min(width, 4096));
    mData.mDepth = std::max(1, std::min(depth, 4096));

    mTexture.touch();
}

void
HxGLTexture3D::setTexture(int size, void* data)
{
    mData.mWidth = size;
    mData.mHeight = size;
    mData.mDepth = size;
    mData.mData = data;

    mTexture.touch();
}

void
HxGLTexture3D::setTextureData(void* data)
{
    mData.mData = data;

    mTexture.touch();
}

void
HxGLTexture3D::unbind(int texunit)
{
    glActiveTexture(GL_TEXTURE0 + texunit);
    glBindTexture(GL_TEXTURE_3D, 0);
    glActiveTexture(GL_TEXTURE0);
}
