#include <mclib/internal/McString.h>

#include <hxcore/HxMessage.h>

#include "HxGLFrameBufferObject.h"

HxGLFrameBufferObject::GLFBO::GLFBO()
    : mBind(false)
    , mEnable(true)
    , mFrameBuffer(0)
    , mFrameTexture(0)
    , mDepthBuffer(0)
    , mDepthTexture(0)
    , mPrevBoundFBO(0)
{
    if (!GLEW_EXT_framebuffer_object)
    {
        theMsg->printf("FBO Error: system has no support for FBOs");
        mEnable = false;
        return;
    }

    glGenFramebuffersEXT(1, &mFrameBuffer);
    glGenTextures(1, &mFrameTexture);
    glGenRenderbuffersEXT(1, &mDepthBuffer);
    glGenTextures(1, &mDepthTexture);

    if (mFrameBuffer  <= 0  ||
        mDepthBuffer  <= 0  ||
        mFrameTexture <= 0 ||
        mDepthTexture <= 0)
    {
        theMsg->printf("FBO Error: could not create FBO or FBO textures");
        mEnable = false;
    }
}




HxGLFrameBufferObject::GLFBO::~GLFBO()
{
    if (mDepthBuffer)  glDeleteRenderbuffersEXT(1, &mDepthBuffer);
    if (mDepthTexture) glDeleteTextures(1, &mDepthTexture);
    if (mFrameBuffer)  glDeleteFramebuffersEXT(1, &mFrameBuffer);
    if (mFrameTexture) glDeleteTextures(1, &mFrameTexture);
}




void HxGLFrameBufferObject::GLFBO::updateResource(void *data)
{
    if (!mEnable) return;

    FBOData* fboData = (FBOData*) data;

    mWidth  = fboData->mWidth;
    mHeight = fboData->mHeight;

    GLint internalFormat = fboData->mImageData.mInternalFormat;
    GLint format         = fboData->mImageData.mFormat;
    GLenum type          = fboData->mImageData.mType;
    GLint minFilter      = fboData->mImageData.mMinFilter;
    GLint magFilter      = fboData->mImageData.mMagFilter;

    glEnable(GL_TEXTURE_2D);

    GLuint preFBO;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, (GLint*)&preFBO);

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mFrameBuffer);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, mDepthBuffer);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, mWidth, mHeight);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, mDepthBuffer);

    glBindTexture(GL_TEXTURE_2D, mFrameTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, mWidth, mHeight, 0, format, type, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, mFrameTexture, 0);

    glBindTexture(GL_TEXTURE_2D, mDepthTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, mWidth, mHeight, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, 0);
    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_TEXTURE_2D, mDepthTexture, 0);

    //check for fbo completeness
    errorFBOCompletenessHandling();

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, preFBO);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
}




void HxGLFrameBufferObject::GLFBO::errorFBOCompletenessHandling()
{
    //bind buffer object
    GLuint preFBO;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, (GLint*)&preFBO);

    if(preFBO != mFrameBuffer) glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mFrameBuffer);

    //check for error, put error to amira console
    GLenum fboStat = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);

    if (fboStat != GL_FRAMEBUFFER_COMPLETE)
    {
        McString errMsg;

        switch (fboStat)
        {
            case GL_FRAMEBUFFER_UNDEFINED:
                errMsg = "OpenGL error: Has a window been defined?";
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
                errMsg = "OpenGL error: Incomplete framebuffer attachment";
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
                errMsg = "OpenGL error: Missing framebuffer attachment";
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
                errMsg = "OpenGL error: Incomplete draw buffer, while creating framebuffer object";
            case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
                errMsg = "OpenGL error: Incomplete read buffer, while creating framebuffer object";
                break;
            case GL_FRAMEBUFFER_UNSUPPORTED:
                errMsg = "OpenGL error: Framebuffer unsupported";
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
                errMsg = "OpenGL error: Incomplete multisample, while creating framebuffer object";
                break;
            case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
                errMsg = "OpenGL error: Incomplete layer targets, while creating framebuffer object";
                break;
        }
        errMsg += " in HxGLFrameBufferObjec";

        mEnable = false;

        theMsg->printf(errMsg.getString());
    }

    //unbind buffer object, restore old fbo
    if(preFBO != mFrameBuffer) glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, preFBO);
}




HxGLFrameBufferObject::HxGLFrameBufferObject(void)
{
    mData.mWidth = 4;
    mData.mHeight = 4;

    mImageTU = GL_TEXTURE0;
    mDepthTU = GL_TEXTURE1;

    mData.mImageData.mInternalFormat = GL_RGBA8;
    mData.mImageData.mFormat         = GL_RGBA;
    mData.mImageData.mType           = GL_UNSIGNED_BYTE;
    mData.mImageData.mMagFilter      = GL_NEAREST;
    mData.mImageData.mMinFilter      = GL_NEAREST;
}




HxGLFrameBufferObject::~HxGLFrameBufferObject(void)
{
}




void HxGLFrameBufferObject::bind(unsigned int contextID, bool depthTest)
{
    GLFBO* fbo = mFBO.getInstance(contextID);

    mFBO.update(contextID, &mData);

    errorHandling(contextID, fbo);

    if (!fbo->mEnable) return;

    glPushAttrib(GL_VIEWPORT_BIT);
    glViewport(0, 0, fbo->mWidth, fbo->mHeight);

    glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, (GLint*)&(fbo->mPrevBoundFBO));
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo->mFrameBuffer);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (depthTest) glEnable(GL_DEPTH_TEST);

    errorHandling(contextID, fbo);

    fbo->mBind = true;
}




void HxGLFrameBufferObject::justBind(unsigned int contextID)
{
    GLFBO* fbo = mFBO.getInstance(contextID);

    mFBO.update(contextID, &mData);

    errorHandling(contextID, fbo);

    if (!fbo->mEnable) return;

    glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, (GLint*)&(fbo->mPrevBoundFBO));
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo->mFrameBuffer);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    errorHandling(contextID, fbo);

    fbo->mBind = true;
}




void HxGLFrameBufferObject::copyFrameBuffer(unsigned int contextID)
{
    GLFBO* fbo = mFBO.getInstance(contextID);

    mFBO.update(contextID, &mData);

    errorHandling(contextID, fbo);

    glEnable(GL_TEXTURE_2D);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, fbo->mFrameTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, fbo->mWidth,
            fbo->mHeight, 0);

    errorHandling(contextID, fbo);

    glBindTexture(GL_TEXTURE_2D, fbo->mDepthTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, 0, 0, fbo->mWidth,
            fbo->mHeight, 0);

    glBindTexture(GL_TEXTURE_2D, 0);

    errorHandling(contextID, fbo);
}




void HxGLFrameBufferObject::end2D()
{
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib();
}




void HxGLFrameBufferObject::errorHandling(unsigned int contextID, GLFBO* fbo)
{
    McString errMsg;
    GLenum err = glGetError();

    if (err != GL_NO_ERROR)
    {
        switch (err)
        {
            case GL_INVALID_ENUM:
                errMsg = "OpenGL error: invalid enum";
            case GL_INVALID_VALUE:
                errMsg = "OpenGL error: invalid value";
            case GL_INVALID_OPERATION:
                errMsg = "OpenGL error: invalid operator";
            case GL_STACK_OVERFLOW:
                errMsg = "OpenGL error: stack overflow";
            case GL_STACK_UNDERFLOW:
                errMsg = "OpenGL error: stack underflow";
            case GL_OUT_OF_MEMORY:
                errMsg = "OpenGL error: out of memory";
        }

        errMsg += McString(0, " in HxGLFrameBufferObject in context: %d", contextID);

        fbo->mEnable = false;

        theMsg->printf(errMsg.getString());
    }
}




int HxGLFrameBufferObject::getHeight() const
{
    return mData.mHeight;
}




int HxGLFrameBufferObject::getWidth() const
{
    return mData.mWidth;
}




void HxGLFrameBufferObject::render(unsigned int contextID)
{
    GLFBO* fbo = mFBO.getInstance(contextID);

    if (!fbo->mEnable) return;

    start2D();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(mImageTU);
    glBindTexture(GL_TEXTURE_2D, fbo->mFrameTexture);
    glActiveTexture(mDepthTU);
    glBindTexture(GL_TEXTURE_2D, fbo->mDepthTexture);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(-1.0f, 1.0f);

    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(1.0f, 1.0f);

    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(1.0f, -1.0f);

    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(-1.0f, -1.0f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(mImageTU);
    glBindTexture(GL_TEXTURE_2D, 0);

    end2D();

    errorHandling(contextID, fbo);
}




void HxGLFrameBufferObject::render(HxGLTexture2D& imageTex, HxGLTexture2D& depthTex, unsigned int contextID)
{
    GLFBO* fbo = mFBO.getInstance(contextID);

    if (!fbo->mEnable) return;

    start2D();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    unsigned int imgTex = imageTex.getTexture(contextID, false);
    unsigned int depTex = depthTex.getTexture(contextID, false);

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, imgTex);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depTex);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 1.0f);
    glVertex2f(-1.0f, 1.0f);

    glTexCoord2f(1.0f, 1.0f);
    glVertex2f(1.0f, 1.0f);

    glTexCoord2f(1.0f, 0.0f);
    glVertex2f(1.0f, -1.0f);

    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(-1.0f, -1.0f);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);

    end2D();

    errorHandling(contextID, fbo);
}




void HxGLFrameBufferObject::start2D()
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();

    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    glLoadIdentity();

    glColor4f(1.0, 1.0, 1.0, 1.0);
    glDisable(GL_LIGHTING);
}




bool HxGLFrameBufferObject::unbind(unsigned int contextID)
{
    GLFBO* fbo = mFBO.getInstance(contextID);

    if (!fbo->mEnable || !fbo->mBind) return false;

    glPopAttrib();

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo->mPrevBoundFBO);

    errorHandling(contextID, fbo);

    fbo->mBind = false;

    return true;
}




bool HxGLFrameBufferObject::justUnbind(unsigned int contextID)
{
    GLFBO* fbo = mFBO.getInstance(contextID);

    if (!fbo->mEnable || !fbo->mBind) return false;

    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo->mPrevBoundFBO);

    errorHandling(contextID, fbo);

    fbo->mBind = false;

    return true;
}




void HxGLFrameBufferObject::resize(int width, int height)
{
    if (mData.mWidth == (GLsizei) width && mData.mHeight == (GLsizei) height) return;

    mData.mWidth = (GLsizei) std::min(4096, std::max(0, width));
    mData.mHeight = (GLsizei) std::min(4096, std::max(0, height));

    mFBO.touch();
}




GLuint HxGLFrameBufferObject::getImageTextureUnit()
{
    return mImageTU;
}




GLuint HxGLFrameBufferObject::getDepthTextureUnit()
{
    return mDepthTU;
}




void HxGLFrameBufferObject::setImageTextureUnit(GLuint imageTU)
{
    mImageTU = imageTU;
}




void HxGLFrameBufferObject::setDepthTextureUnit(GLuint depthTU)
{
    mDepthTU = depthTU;
}




GLuint HxGLFrameBufferObject::getImageTexture(unsigned int contextID)
{
    GLuint ret = 0;
    GLFBO* fbo = mFBO.getInstance(contextID);

    if (fbo) ret = fbo->mFrameTexture;

    return ret;
}




GLuint HxGLFrameBufferObject::getDepthTexture(unsigned int contextID)
{

    GLuint ret = 0;
    GLFBO* fbo = mFBO.getInstance(contextID);

    if (fbo) ret = fbo->mDepthTexture;

    return ret;
}




void HxGLFrameBufferObject::setImageFormat(GLint internalFormat, GLint format)
{
    mData.mImageData.mInternalFormat = internalFormat;
    mData.mImageData.mFormat         = format;

    mFBO.touch();
}




void HxGLFrameBufferObject::setImageFetchMode(GLint mode)
{
    if (mode != GL_NEAREST && mode != GL_LINEAR) return;

    mData.mImageData.mMagFilter = mode;
    mData.mImageData.mMinFilter = mode;

    mFBO.touch();
}




void HxGLFrameBufferObject::setImageType(GLenum type)
{
    mData.mImageData.mType = type;
    mFBO.touch();
}



