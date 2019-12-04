#ifndef HX_GL_UTILS_GL_COLORMAP_H
#define HX_GL_UTILS_GL_COLORMAP_H

#include <hxglutils/api.h>
#include <hxglutils/HxContextCache.h>

class HxPortColormap;

class HXGLUTILS_API HxGLColormap
{

    // subclasses
    private:

        /**
            Subclass that handles for each GPU a one dimensional
            texture for the colormap. The texture will automatically
            updated while changing the colormap.
        */
        class GLColormap
        {
            public:

                 GLColormap();
                ~GLColormap();

            public:

                void updateResource(void* data);

            public:

                // GPU data
                GLuint mTexture;
        };


        /**
            Color map data
        */
        struct ColormapData
        {
            unsigned char* mColors;
            int            mSize;
        };



    // constructors and destructor
    public:

                 /// default constructor
                 HxGLColormap(void);


                 /// default destructor
        virtual ~HxGLColormap(void);


    // methods
    public:

        /**
            Unbinds the texture and enables two dimensional textures.
        */
        void disable();

        /**
            Enables the colormap texture for a context and
            binds it at position 0.
        */
        void enable(unsigned int contextID);

        /**
            Fills the colormap with values given by current colormap
            port "colmap". This method does not resize the colormap
            texture.
        */
        void fill(HxPortColormap* colmap, int size);

        /**
            Fills the colormap with a constant color.
        */
        void fill(float* rgb);

        /**
            Fills the colormap with a constant color.
        */
        void fill(float red, float green, float blue);

        /**
            Fills the colormap with random values.
        */
        void fillRandom();

        /**
            Sets a color for index
        */
        void setColor(float* rgb, int index);

        /**
            Sets the minimum and maximum value.
        */
        void setMinMax(float min, float max);

        float getMin() const { return mMinValue; }
        float getMax() const { return mMaxValue; }


        /**
            Sets the size of the colormap. Only values of 2^n are possible.
        */
        void setSize(int size);
        int getSize() const { return mData.mSize; }
        /**
            Clones a colormap.
        */
        HxGLColormap& operator=(const HxGLColormap& colmap);

    // attributes
    private:

        HxContextCache < GLColormap > mColormap;
        ColormapData                  mData;

        float                         mMinValue;
        float                         mMaxValue;
};



#endif
