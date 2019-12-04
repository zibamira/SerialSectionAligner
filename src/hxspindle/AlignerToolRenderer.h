#ifndef HXSPINDLE_ALIGNERTOOLRENDERER_H
#define HXSPINDLE_ALIGNERTOOLRENDERER_H

#include <hxspindle/api.h>
#include <hxspindle/HxSerialSectionStack.h>

#include <hxfield/HxUniformScalarField3.h>

#include <mcgl/internal/mcgl.h>

#include <hxglutils/HxGLFrameBufferObject.h>
#include <hxglutils/HxGLShader.h>
#include <hxglutils/HxGLTexture2D.h>

#include <Inventor/SbMatrix.h>
#include <Inventor/SbVec.h>




/**
    Render a slice of the field data.
*/
class HXSPINDLE_API AlignerToolRenderer
{
    public:


        AlignerToolRenderer(void);


    public:


        /**
            Sets the position of the view (-1...1).
        */
        void setPosition(float x, float y);




        /**
            Sets the size of the view (0...2).
        */
        void setSize(float width, float height);


    protected:


        /**
            Returns the next larger or equal number of power
            of two than the given number. Some textures still
            require power of two.
        */
        GLuint getNextPowerOfTwo(GLuint number);




        /**
            Render a texture into the tool window.
        */
        void renderTexture(GLuint textureID, float width = 1.0f, float height = 1.0f) const;



        /**
            Render a texture into the tool window.
        */
        void renderTexture(GLuint textureID, bool transparent, float width = 1.0f, float height = 1.0f) const;



        /**
            Render a texture into the tool window.
        */
        void renderTextureDepth(GLuint textureID, GLuint depthID, float width = 1.0f, float height = 1.0f);




        /**
            Render a texture into the tool window.
        */
        void renderTexture(HxGLTexture2D& texture, unsigned int contextID);




        /**
            Updates the size of the frame buffer object depending
            on the viewer size and the size of the tool window.
            The method returns false if the update failed.
        */
        bool updateFrameBuffer();




        /**
            Computes viewer pixel coordinates into tool window coordinates
            (-1...1, -1...1). The method returns false if the viewer
            coordinate is outside the tool window.
        */
        bool windowToToolCoord(const SbVec2s viewerPosition, SbVec2f& toolPosition) const;


    protected:


        HxGLFrameBufferObject      mFrameBuffer;

        SbMatrixd                  mMatrixModelView;
        SbMatrixd                  mMatrixProjection;
        SbVec2f                    mRenderPosition;
        SbVec2f                    mRenderSize;
};




#endif

