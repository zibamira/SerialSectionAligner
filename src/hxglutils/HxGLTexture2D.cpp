#include "HxGLTexture2D.h"

void
HxGLTexture2D::GLTexture2D::updateResource(void* data)
{
    Texture2DData* tData = (Texture2DData*)data;

    glEnable(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, mTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, tData->mMagFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, tData->mMinFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, tData->mWrap_S);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, tData->mWrap_T);

    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 tData->mInternalFormat,
                 tData->mWidth,
                 tData->mHeight,
                 tData->mBorder,
                 tData->mFormat,
                 tData->mType,
                 tData->mData);

    glBindTexture(GL_TEXTURE_2D, 0);

    glDisable(GL_TEXTURE_2D);
}

HxGLTexture2D::HxGLTexture2D(void)
{
}

HxGLTexture2D::~HxGLTexture2D(void)
{
}

void
HxGLTexture2D::activate(unsigned int contextID)
{
    mTexture.update(contextID, &mData);
}

void
HxGLTexture2D::bind(unsigned int contextID, int texunit)
{
    mTexture.update(contextID, &mData);

    GLTexture2D* texture = mTexture.getInstance(contextID);

    bool set3D = glIsEnabled(GL_TEXTURE_3D);
    GLint activeTex = GL_TEXTURE0;

    glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTex);

    if (set3D)
        glDisable(GL_TEXTURE_3D);

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0 + texunit);
    glBindTexture(GL_TEXTURE_2D, texture->mTexture);
    glActiveTexture((GLenum)activeTex);
    glDisable(GL_TEXTURE_2D);

    if (set3D)
        glEnable(GL_TEXTURE_3D);
}

void
HxGLTexture2D::copyColorFrameBuffer(unsigned int contextID)
{
    mData.mInternalFormat = GL_RGBA8;
    mData.mFormat = GL_RGBA;
    mData.mType = GL_UNSIGNED_BYTE;

    mTexture.update(contextID, &mData);

    GLTexture2D* texture = mTexture.getInstance(contextID);

    glEnable(GL_TEXTURE_2D);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture->mTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, mData.mWidth, mData.mHeight, 0);

    glBindTexture(GL_TEXTURE_2D, 0);

    glDisable(GL_TEXTURE_2D);
}

void
HxGLTexture2D::copyDepthFrameBuffer(unsigned int contextID)
{
    mData.mInternalFormat = GL_DEPTH_COMPONENT24;
    mData.mFormat = GL_DEPTH_COMPONENT;
    mData.mType = GL_FLOAT;

    mTexture.update(contextID, &mData);

    GLTexture2D* texture = mTexture.getInstance(contextID);

    glEnable(GL_TEXTURE_2D);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture->mTexture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, mData.mMinFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mData.mMagFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, mData.mWrap_S);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, mData.mWrap_T);

    glCopyTexImage2D(GL_TEXTURE_2D, 0, mData.mInternalFormat, 0, 0, mData.mWidth, mData.mHeight, 0);

    glBindTexture(GL_TEXTURE_2D, 0);

    glDisable(GL_TEXTURE_2D);
}

GLuint
HxGLTexture2D::getTexture(unsigned int contextID, bool update)
{
    if (update)
        mTexture.update(contextID, &mData);

    GLTexture2D* texture = mTexture.getInstance(contextID);

    return texture->mTexture;
}

void
HxGLTexture2D::setFetchMode(GLint mode)
{
    if (mode != GL_NEAREST && mode != GL_LINEAR)
        return;

    mData.mMagFilter = mode;
    mData.mMinFilter = mode;

    mTexture.touch();
}

void
HxGLTexture2D::setFormat(GLint internalFormat, GLenum format, GLenum type)
{
    mData.mInternalFormat = internalFormat;
    mData.mFormat = format;
    mData.mType = type;

    mTexture.touch();
}

void
HxGLTexture2D::setWrap(GLint param)
{
    mData.mWrap_S = param;
    mData.mWrap_T = param;

    mTexture.touch();
}

void
HxGLTexture2D::setSize(GLsizei width, GLsizei height)
{
    if (mData.mWidth == width && mData.mHeight == height)
        return;

    mData.mHeight = std::max(1, std::min(height, 4096));
    mData.mWidth = std::max(1, std::min(width, 4096));

    mTexture.touch();
}

void
HxGLTexture2D::setTexture(int size, void* data)
{
    mData.mWidth = size;
    mData.mHeight = size;
    mData.mData = data;

    mTexture.touch();
}

void
HxGLTexture2D::setTextureData(void* data)
{
    mData.mData = data;

    mTexture.touch();
}

void
HxGLTexture2D::unbind(int texunit)
{
    bool set3D = glIsEnabled(GL_TEXTURE_3D);
    GLint activeTex = GL_TEXTURE0;

    if (set3D)
        glDisable(GL_TEXTURE_3D);

    glGetIntegerv(GL_ACTIVE_TEXTURE, &activeTex);

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0 + texunit);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture((GLenum)activeTex);
    glDisable(GL_TEXTURE_2D);

    if (set3D)
        glEnable(GL_TEXTURE_3D);
}

int
HxGLTexture2D::getWidth() const
{
    return mData.mWidth;
}

int
HxGLTexture2D::getHeight() const
{
    return mData.mHeight;
}
