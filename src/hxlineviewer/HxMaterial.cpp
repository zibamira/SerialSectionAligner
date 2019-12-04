#include "HxMaterial.h"





HxMaterial::HxMaterial(void)
{
    setDefault();
}




HxMaterial::~HxMaterial(void)
{
}




void HxMaterial::setDefault()
{
    mAmbient        = true;
    mAmbientScale   = 0.1;

    mDiffuse        = true;
    mDiffuseScale   = 1.0;

    mSpecular       = true;
    mSpecularScale  = 0.8;
    mShininess      = 40.0;
}




void HxMaterial::setShaderParameter(unsigned int contextID, HxGLShader& shader)
{
    float mat[4];

    mat[0] = (float) mAmbient  * (float) mAmbientScale;
    mat[1] = (float) mDiffuse  * (float) mDiffuseScale;
    mat[2] = (float) mSpecular * (float) mSpecularScale;
    mat[3] = (float) mShininess;

    shader.setParameter4f(contextID, "material", mat);
}


