#ifndef HX_GL_UTILS_GL_TEXTURE_2D_H
#define HX_GL_UTILS_GL_TEXTURE_2D_H

#include <hxglutils/HxGLTexture1D.h>

class HXGLUTILS_API HxGLTexture2D
{
public:
    struct GLTexture2D : public HxGLTexture1D::GLTexture1D
    {
        void updateResource(void* data);
    };

    struct Texture2DData : public HxGLTexture1D::Texture1DData
    {
        Texture2DData()
            : HxGLTexture1D::Texture1DData()
            , mHeight(0)
            , mWrap_T(GL_CLAMP_TO_EDGE)
        {
        }

        GLsizei mHeight;
        GLint mWrap_T;
    };

    // constructors and destructor
public:
    /// default constructor
    HxGLTexture2D(void);

    /// default destructor
    virtual ~HxGLTexture2D(void);

    // methods
public:
    /**
            WORKAROUND. Some modules need to call update twice.
            We need to find a better solution.
        */
    void activate(unsigned int contextID);

    /**
            Binds a texture to a shader. The texture unit must be between
            0 and GL_MAX_TEXTURE_UNITS.
        */
    void bind(unsigned int contextID, int texunit);

    /**
            Copy color from frame buffer to the texture. The method
            keeps the texture properties except the format attributes.
            Afterwards the texture has RGBA format (internal 8 bit per
            channel as unsigned byte).
        */
    void copyColorFrameBuffer(unsigned int contextID);

    /**
            Copy depth from frame buffer to the texture. The method
            keeps the texture properties except the format attributes.
            Afterwards the texture has DPETH_COMPONENT format (internal
            24 bits as float).
        */
    void copyDepthFrameBuffer(unsigned int contextID);

    /**
            Updates the texture (if necessary) and returns
            the texture handle.
        */
    GLuint getTexture(unsigned int contextID, bool update = true);

    /**
            Sets texture wrap parameter (GL_CLAMP_TO_EDGE per default).
        */
    void setWrap(GLint param);

    /**
            Sets the texture format (GL_RGBA, GL_RGBA8, GL_UNSIGNED_BYTE
            per default).
        */
    void setFormat(GLint internalFormat, GLenum format, GLenum type);

    /**
            Sets the fetch mode (GL_NEAREST or GL_LINEAR, GL_NEAREST per default).
        */
    void setFetchMode(GLint mode);

    /**
            Sets the size of a texture.
        */
    void setSize(GLsizei width, GLsizei height);

    /**
            Sets a quadratic texture, where size is the number of pixels in
            one dimension.
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

    int getWidth() const;
    int getHeight() const;

private:
    // attributes
private:
    Texture2DData mData;
    HxContextCache<GLTexture2D> mTexture;
};

#endif
