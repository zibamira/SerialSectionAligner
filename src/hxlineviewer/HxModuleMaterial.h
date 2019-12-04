#ifndef HXGLUTILS_HXMODULEMATERIAL_H
#define HXGLUTILS_HXMODULEMATERIAL_H

#include <hxlineviewer/api.h>

#include <hxcore/HxModule.h>
#include <hxcore/HxPortFloatSlider.h>
#include <hxcore/HxPortToggleList.h>
#include <hxcore/HxPortSeparator.h>

#include <hxlineviewer/HxMaterial.h>


class HXLINEVIEWER_API HxModuleMaterial : public HxModule
{
    HX_HEADER(HxModuleMaterial);

    // methods
    public:

        /**
            Computations.
        */
        virtual void compute();

        /**
            Returns the material.
        */
        HxMaterial getMaterial();


        /**
            Update GUI.
        */
        void update();


    // ports
    public :

        HxPortToggleList    mPortMode;

        HxPortSeparator     mPortAmbientSeparator;
        HxPortFloatSlider   mPortAmbientScale;

        HxPortSeparator     mPortDiffuseSeparator;
        HxPortFloatSlider   mPortDiffuseScale;

        HxPortSeparator     mPortSpecularSeparator;
        HxPortFloatSlider   mPortSpecularScale;
        HxPortFloatSlider   mPortShininess;


    // attributes
    private:

        HxMaterial          mMaterial;

};


#endif
