#ifndef HX_GL_UTILS_GL_TEXTURE_3D_H
#define HX_GL_UTILS_GL_TEXTURE_3D_H

#include "HxGLTexture2D.h"

class HXGLUTILS_API HxGLTexture3D
{
public:
    struct GLTexture3D : public HxGLTexture2D::GLTexture2D
    {
        void updateResource(void* data);
    };

    struct Texture3DData : public HxGLTexture2D::Texture2DData
    {
        Texture3DData()
            : HxGLTexture2D::Texture2DData()
            , mDepth(0)
            , mWrap_R(GL_CLAMP_TO_EDGE)
        {
        }

        GLsizei mDepth;
        GLint mWrap_R;
    };

    // constructors and destructor
public:
    /// default constructor
    HxGLTexture3D(void);

    /// default destructor
    virtual ~HxGLTexture3D(void);

    // methods
public:
    /**
            WORKAROUND. Some modules need to call update twice.
            We need to find a better solution.
        */
    void activate(unsigned int contextID);

    /**
            Binds a texture to a shader.
        */
    void bind(unsigned int contextID, int texunit);

    /**
            Updates the texture (if necessary) and returns
            the texture handle.
        */
    GLuint getTexture(unsigned int contextID, bool update = true);

    /**
            Sets the texture format.
        */
    void setFormat(GLint internalFormat, GLenum format, GLenum type);

    /**
            Sets the fetch mode (GL_NEAREST or GL_LINEAR, GL_NEAREST per default).
        */
    void setFetchMode(GLint mode);

    /**
            Sets the size of a texture.
        */
    void setSize(GLsizei width, GLsizei height, GLsizei depth);

    /**
            Sets a cubic texture.
        */
    void setTexture(int size, void* data);

    /**
            Sets texture data.
        */
    void setTextureData(void* data);

    /**
            Unbinds the texture.
        */
    void unbind(int texunit);

    // attributes
private:
    Texture3DData mData;
    HxContextCache<GLTexture3D> mTexture;
};

#endif
