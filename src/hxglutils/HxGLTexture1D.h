#ifndef HX_GL_UTILS_GL_TEXTURE_1D_H
#define HX_GL_UTILS_GL_TEXTURE_1D_H

#include <mcgl/internal/mcgl.h>

#include <hxglutils/api.h>
#include <hxglutils/HxContextCache.h>
#include <hxglutils/HxGLShader.h>

class HXGLUTILS_API HxGLTexture1D
{

    public:

        struct HXGLUTILS_API GLTexture1D
        {
             GLTexture1D();
            ~GLTexture1D();

            void updateResource(void* data);

            GLuint mTexture;
        };

        struct Texture1DData
        {
            Texture1DData()
            :   mBorder           (0)
            ,   mData             (0)
            ,   mFormat           (GL_RGBA)
            ,   mWidth            (0)
            ,   mInternalFormat   (GL_RGBA8)
            ,   mMagFilter        (GL_NEAREST)
            ,   mMinFilter        (GL_NEAREST)
            ,   mType             (GL_UNSIGNED_BYTE)
            ,   mWrap_S           (GL_CLAMP_TO_EDGE)
            {
            }

            GLint    mBorder;
            void*    mData;
            GLenum   mFormat;
            GLsizei  mWidth;
            GLint    mInternalFormat;
            GLint    mMagFilter;
            GLint    mMinFilter;
            GLenum   mType;
            GLint    mWrap_S;
        };



    // constructors and destructor
    public:

                 /// default constructor
                 HxGLTexture1D(void);


                 /// default destructor
        virtual ~HxGLTexture1D(void);



    // methods
    public:

        /**
            Binds a texture to a shader.
        */
        void bind(unsigned int contextID, HxGLShader& shader, int texunit, const char* name);

        /**
            Copy color from frame buffer to the texture.
        */
        void copyColorFrameBuffer(unsigned int contextID);

        /**
            Copy depth from frame buffer to the texture.
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
        void setSize(GLsizei width);

        /**
            Sets texture data.
        */
        void setTextureData(void *data);

        /**
            Unbinds the texture.
        */
        void unbind(int texunit);

    // attributes
    private:

        Texture1DData                   mData;
        HxContextCache < GLTexture1D >  mTexture;
};







#endif
