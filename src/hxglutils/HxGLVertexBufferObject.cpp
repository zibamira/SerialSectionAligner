#include <hxcore/HxMessage.h>

#include <hxglutils/api.h>

#include "HxGLVertexBufferObject.h"

#define BUFFER_OFFSET(i) ((char *)NULL + (i))

HxGLVertexBufferObject::GLVBO::GLVBO()
    : mNumVerts(0)
    , mVertSize(0)
    , mVBO(0)
    , mTarget(GL_ARRAY_BUFFER)
    , mEnable(true)
{
    if (!GLEW_ARB_vertex_buffer_object)
    {
        theMsg->printf("VBO Error: system has no support for VBOs");
        mEnable = false;
        return;
    }
}




HxGLVertexBufferObject::GLVBO::~GLVBO()
{
    if (mVBO) glDeleteBuffers(1, &mVBO);
}




void HxGLVertexBufferObject::GLVBO::updateResource(void *data)
{
    if (!mEnable) return;

    if (!mVBO) glGenBuffers(1, &mVBO);

    // test if buffer was generated

    if (mVBO <= 0)
    {
        theMsg->printf("VBO Error: could not create VBO");
        mEnable = false;
    }

    VBOData* vData = (VBOData*) data;

    // only rough check, should be improved

    if (vData->mVertexSize * vData->mNumVertices < 0)
    {
        theMsg->printf("VBO Error: size overflow");
        mEnable = false;
    }

    GLsizeiptr oldTotalSize = mNumVerts * mVertSize;

    mNumVerts = vData->mNumVertices;
    mVertSize = vData->mVertexSize;
    mTarget   = vData->mTarget;

    // remove empty buffer and return

    if (mNumVerts == 0 || mVertSize == 0)
    {
        if (mVBO) glDeleteBuffers(1, &mVBO);
        mVBO = 0;
        return;
    }

    // fill buffer with data

    GLsizeiptr totalSize = mNumVerts * mVertSize;

    glBindBufferARB(mTarget, mVBO);

    //create new buffer or fill old

    if(totalSize != oldTotalSize)
    {
        glBufferDataARB(mTarget, totalSize, vData->mData, GL_DYNAMIC_DRAW);
        mVertSize = vData->mVertexSize;
        mNumVerts = vData->mNumVertices;
    }
    else
    {
        glBufferSubDataARB(mTarget, 0, totalSize, vData->mData);
    }

    int error = glGetError();

    if (error == GL_OUT_OF_MEMORY)
    {
        theMsg->printf("VBO Error: out of memory");
        mEnable = false;
    }
    else if (error != 0)
    {
        theMsg->printf("VBO Error: %d", error);
        mEnable = false;
    }

    glBindBufferARB(mTarget, 0);
}





HxGLVertexBufferObject::HxGLVertexBufferObject(void)
{
    mData.mData         = 0;
    mData.mNumVertices  = 0;
    mData.mVertexSize   = 0;
    mData.mTarget       = GL_ARRAY_BUFFER;
}





HxGLVertexBufferObject::~HxGLVertexBufferObject(void)
{
}




void HxGLVertexBufferObject::disableAttribPointer(unsigned int contextID, int att, GLint size, GLenum dataTyp)
{
    GLint typeSize = getTypeSize(dataTyp);

    if (typeSize == 0)        return;
    if (size < 1 || size > 4) return;

    int numAtt = (int) (mData.mVertexSize / (typeSize * size));

    if (mData.mVertexSize % (typeSize * size) != 0) numAtt++;

    GLVBO* vbo = mVBO.getInstance(contextID);

    if (!vbo->mEnable) return;

    glBindBufferARB(vbo->mTarget, 0);

    for (int i = numAtt - 1; i >= 0; --i)
    {
        glDisableVertexAttribArray(i + att);
    }
}




void HxGLVertexBufferObject::disableVertexAndTexCoordPointer(unsigned int contextID, GLint size, GLenum dataTyp)
{
    GLint typeSize = getTypeSize(dataTyp);

    if (typeSize == 0)        return;
    if (size < 1 || size > 4) return;

    int numTextures = (int) (mData.mVertexSize / (typeSize * size));

    if (mData.mVertexSize % (typeSize * size) == 0) numTextures--;

    GLVBO* vbo = mVBO.getInstance(contextID);

    if (!vbo->mEnable) return;

    glBindBufferARB(vbo->mTarget, 0);
    glDisableClientState(GL_VERTEX_ARRAY);

    for (int i = numTextures - 1; i >= 0; --i)
    {
        glClientActiveTextureARB(GL_TEXTURE0_ARB + i);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    glClientActiveTextureARB(GL_TEXTURE0_ARB);
}




void HxGLVertexBufferObject::disableTexCoordPointer(unsigned int contextID, int tex, GLint size, GLenum dataTyp)
{
    GLint typeSize = getTypeSize(dataTyp);

    if (typeSize == 0)        return;
    if (size < 1 || size > 4) return;

    int numTextures = (int) (mData.mVertexSize / (typeSize * size));

    if (mData.mVertexSize % (typeSize * size) != 0) numTextures++;

    GLVBO* vbo = mVBO.getInstance(contextID);

    if (!vbo->mEnable) return;

    glBindBufferARB(vbo->mTarget, 0);

    for (int i = numTextures - 1; i >= 0; --i)
    {
        glClientActiveTextureARB(GL_TEXTURE0_ARB + i + tex);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    glClientActiveTextureARB(GL_TEXTURE0_ARB);
}




bool HxGLVertexBufferObject::enableAttribPointer(unsigned int contextID, int att, GLint size, GLenum dataTyp)
{
    GLint typeSize  = getTypeSize(dataTyp);
    GLint shiftSize = typeSize * size;

    if (typeSize == 0)        return false;
    if (size < 1 || size > 4) return false;

    int numAtt = (int) mData.mVertexSize / shiftSize;

    if (mData.mVertexSize % shiftSize != 0) numAtt++;

    mVBO.update(contextID, &mData);

    GLVBO* vbo = mVBO.getInstance(contextID);

    if (!vbo->mEnable) return false;

    glBindBufferARB(vbo->mTarget, vbo->mVBO);

    for (int i = 0; i < numAtt; ++i)
    {
        glEnableVertexAttribArray(i + att);
        glVertexAttribPointer(i + att, size, dataTyp, GL_FALSE, (GLsizei) mData.mVertexSize, BUFFER_OFFSET((i * shiftSize)));
    }

    return true;
}

/*
 * NOT TESTED
 * TODO test method
 */
bool HxGLVertexBufferObject::enableSingleAttribPointer(unsigned int contextID, int att, GLint size, GLenum dataTyp, const GLvoid* pointer){

       if (size < 1 || size > 4) return false;


       mVBO.update(contextID, &mData);

       GLVBO* vbo = mVBO.getInstance(contextID);

       if (!vbo->mEnable) return false;

       glBindBufferARB(vbo->mTarget, vbo->mVBO);


       glEnableVertexAttribArray(att);
       glVertexAttribPointer(att, size, dataTyp, GL_FALSE, (GLsizei) mData.mVertexSize, pointer);


       return true;
}




bool HxGLVertexBufferObject::enableTexCoordPointer(unsigned int contextID, int tex, GLint size, GLenum dataTyp)
{
    GLint typeSize  = getTypeSize(dataTyp);
    GLint shiftSize = typeSize * size;

    if (typeSize == 0)        return false;
    if (size < 1 || size > 4) return false;

    int numTextures = (int) mData.mVertexSize / shiftSize;

    if (mData.mVertexSize % shiftSize != 0) numTextures++;

    mVBO.update(contextID, &mData);

    GLVBO* vbo = mVBO.getInstance(contextID);

    if (!vbo->mEnable) return false;

    // sets the pointers

    glBindBufferARB(vbo->mTarget, vbo->mVBO);

    for (int i = 0; i < numTextures; ++i)
    {
        glClientActiveTextureARB(GL_TEXTURE0_ARB + tex + i);
        glTexCoordPointer(size, dataTyp, (GLsizei) mData.mVertexSize, BUFFER_OFFSET((i * shiftSize)));
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    return true;
}




bool HxGLVertexBufferObject::enableVertexAndTexCoordPointer(unsigned int contextID, GLint size, GLenum dataTyp)
{
    // compute num textures
    GLint typeSize  = getTypeSize(dataTyp);
    GLint shiftSize = typeSize * size;

    if (typeSize == 0)          return false;
    if (size < 1 || size > 4)   return false;

    int numTextures = (int) (mData.mVertexSize / shiftSize);

    if (mData.mVertexSize % shiftSize == 0) numTextures--;

    mVBO.update(contextID, &mData);

    GLVBO* vbo = mVBO.getInstance(contextID);

    // check vbo
    if (!vbo->mEnable) return false;

    // sets the pointers
    glBindBufferARB(vbo->mTarget, vbo->mVBO);

    for (int i = 0; i < numTextures; ++i)
    {
        glClientActiveTextureARB(GL_TEXTURE0_ARB + i);
        glTexCoordPointer(size, dataTyp, (GLsizei) mData.mVertexSize, BUFFER_OFFSET(shiftSize + (i * shiftSize)));
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    }

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(size, dataTyp, (GLsizei) mData.mVertexSize, 0);

    return true;
}




int HxGLVertexBufferObject::getNumVertices()
{
    return (int) mData.mNumVertices;
}


GLint HxGLVertexBufferObject::getTypeSize(GLenum type)
{
    if      (type == GL_FLOAT)  return (GLint) sizeof(GLfloat);
    else if (type == GL_INT)    return (GLint) sizeof(GLint);
    else if (type == GL_SHORT)  return (GLint) sizeof(GLshort);
    else if (type == GL_DOUBLE) return (GLint) sizeof(GLdouble);
    else                        return 0;
}




void HxGLVertexBufferObject::setBuffer(int numVertices, int vertexSize, void *data, GLenum target)
{
    mData.mData             = data;
    mData.mNumVertices      = numVertices;
    mData.mVertexSize       = vertexSize;
    mData.mTarget           = target;

    mVBO.touch();

}
