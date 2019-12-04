#ifndef HXGLUTILS_HXMATERIAL_H
#define HXGLUTILS_HXMATERIAL_H

#include <hxglutils/HxGLShader.h>


class HxMaterial
{

    // Constructor/Destructor
    public:


                 HxMaterial(void);
        virtual ~HxMaterial(void);


    // Methods
    public:

        void setDefault();

        /**
            Sets the material properties as shader parameters.
        */
        void setShaderParameter(unsigned int contextID, HxGLShader& shader);


    // Attributes
    public :

        bool    mAmbient;
        double  mAmbientScale;

        bool    mDiffuse;
        double  mDiffuseScale;

        bool    mSpecular;
        double  mSpecularScale;
        double  mShininess;

};


#endif
