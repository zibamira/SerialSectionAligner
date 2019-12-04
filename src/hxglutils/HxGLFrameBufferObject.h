#ifndef HX_GL_UTILS_GL_FRAME_BUFFER_OBJECT_H
#define HX_GL_UTILS_GL_FRAME_BUFFER_OBJECT_H

#include <mcgl/internal/mcgl.h>

#include <hxglutils/api.h>
#include <hxglutils/HxContextCache.h>
#include <hxglutils/HxGLTexture2D.h>

class HXGLUTILS_API HxGLFrameBufferObject
{



    private:


        class GLFBO
        {

            public:

                 GLFBO();
                ~GLFBO();

            public:

                void updateResource(void* data);


            private:
                /*
                 * Check fbo for completeness
                 * If not complete, put error message to amira console and deactivate incomplete fbo
                 */
                void errorFBOCompletenessHandling();

            public:

                bool    mBind;
                bool    mEnable;
                GLuint  mFrameBuffer;
                GLuint  mFrameTexture;
                GLuint  mDepthBuffer;
                GLuint  mDepthTexture;



                //member variable for restoring previously bound fbo
                GLuint  mPrevBoundFBO;

                GLsizei mHeight;
                GLsizei mWidth;




        };


        //global texture properties
        struct TextureData
        {
            GLint  mInternalFormat;
            GLint  mFormat;
            GLenum mType;
            GLint  mMagFilter;
            GLint  mMinFilter;
        };

        //global fbo properties
        struct FBOData
        {
            GLsizei mHeight;
            GLsizei mWidth;

            //Image texture properties
            TextureData mImageData;
        };




    // constructors and destructor
    public:

                 /// default constructor
                 HxGLFrameBufferObject(void);


                /// default destructor
        virtual ~HxGLFrameBufferObject(void);



    // methods
    public:



        /**
         * Bind FBO: All subsequent draw commands will be directly rendered into FBO textures
         *           Unbind FBO to stop rendering to this FBO
         */
        // FIXME: depthTest
        void bind(unsigned int contextID, bool depthTest = true);

        void justBind(unsigned int contextID);
        bool justUnbind(unsigned int contextID);


        /**
         * Get Image/Depth texture for a specific context
         */
        GLuint getImageTexture(unsigned int contextID);
        GLuint getDepthTexture(unsigned int contextID);


        /**
         * Set Image texture properties
         * PARAMETERS MUST BE VALID TEXTURE PROPERTIES, NOT EXPLICITLY CHECKED
         */
        void setImageFormat(GLint internalFormat, GLint format = GL_RGBA); //(internal)Format, GL_RGBA8(GL_RGBA) by default
        void setImageFetchMode(GLint mode); //fetchMode (GL_NAREST or GL_LINEAR), GL_NEAREST by default
        void setImageType(GLenum type); // type, GL_UNSIGNED_BYTE per default


        /**
            Returns the height of the frame buffer object.
        */
        int getHeight() const;


        /**
            Returns the width of the frame buffer object.
        */
        int getWidth() const;

        /**
         * Copy framebuffer content into Image/Depth texture
         * Ignores texture properties like internalFormat or mag/min-filter
         */
        void copyFrameBuffer(unsigned int contextID);


        /**
         * Render methods: Render texture content to screen
         */
        void render(unsigned int contextID);
        void render(HxGLTexture2D& imageTex, HxGLTexture2D& depthTex, unsigned int contextID);

        void resize(int width, int height);

        /**
            If you call this method without a call of bind(), false
            will be returned.
        */
        bool unbind(unsigned int contextID);


        //Getter for texture units
        GLuint getImageTextureUnit();
        GLuint getDepthTextureUnit();

        /*
         * Set texture units used in render method
         * NUMBER OF SUPPORTED TEXTURE UNITS IS CONTEXT DEPENDENT
         * PARAMETER MUST BE ON RANGE [0, GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS)
         * Typically, 8-16 texture units are supported by graphic hardware
         */
        void setImageTextureUnit(GLuint imageTU);
        void setDepthTextureUnit(GLuint depthTU);

    private:

        void errorHandling(unsigned int contextID, GLFBO* fbo);

        void start2D();

        void end2D();


    // attributes
    private:

        FBOData                   mData;
        HxContextCache < GLFBO >  mFBO;

        //Texture units
        GLuint                    mImageTU;
        GLuint                    mDepthTU;

};







#endif
