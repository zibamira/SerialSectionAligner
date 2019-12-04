#include "HxModuleMaterial.h"

HX_INIT_CLASS(HxModuleMaterial, HxModule)




HxModuleMaterial::HxModuleMaterial(void)
    : mPortMode                (this, "mode", tr("Mode"), 3)
    , mPortAmbientSeparator    (this, "ambient", tr("Ambient"), false)
    , mPortAmbientScale        (this, "ambientScale", tr("Ambient Scale"))
    , mPortDiffuseSeparator    (this, "diffuse", tr("Diffuse"), false)
    , mPortDiffuseScale        (this, "diffuseScale", tr("Diffuse Scale"))
    , mPortSpecularSeparator   (this, "specular", tr("Specular"), false)
    , mPortSpecularScale       (this, "specularScale", tr("Specular Scale"))
    , mPortShininess           (this, "shininess", tr("Shininess"))
{
    mPortMode.setLabel(0, "ambient");
    mPortMode.setLabel(1, "diffuse");
    mPortMode.setLabel(2, "specular");

    mPortMode.setValue(0, 1);
    mPortMode.setValue(1, 1);
    mPortMode.setValue(2, 1);

    mPortAmbientScale.setMinMax(0.0, 1.0);
    mPortAmbientScale.setValue(0.1);

    mPortDiffuseScale.setMinMax(0.0, 2.0);
    mPortDiffuseScale.setValue(1.0);

    mPortSpecularScale.setMinMax(0.0, 1.0);
    mPortSpecularScale.setValue(0.8);
    mPortShininess.setMinMax(5.0, 70.0);
    mPortShininess.setValue(40.0);
}




HxModuleMaterial::~HxModuleMaterial(void)
{
}




void HxModuleMaterial::compute()
{
    if (mPortMode.isNew())
    {
        mMaterial.mAmbient  = (bool) mPortMode.getValue(0);
        mMaterial.mDiffuse  = (bool) mPortMode.getValue(1);
        mMaterial.mSpecular = (bool) mPortMode.getValue(2);
    }

    if (mPortAmbientScale.isNew())
    {
        mMaterial.mAmbientScale = mPortAmbientScale.getValue();
    }

    if (mPortDiffuseScale.isNew())
    {
        mMaterial.mDiffuseScale = mPortDiffuseScale.getValue();
    }

    if (mPortSpecularScale.isNew())
    {
        mMaterial.mSpecularScale = mPortSpecularScale.getValue();
    }

    if (mPortShininess.isNew())
    {
        mMaterial.mShininess = mPortShininess.getValue();
    }
}




HxMaterial HxModuleMaterial::getMaterial()
{
    return mMaterial;
}




void HxModuleMaterial::update()
{
    // ambient

    if (mPortMode.isItemNew(0))
    {
        if (mPortMode.getValue(0) == 0)
        {
            mPortAmbientSeparator.hide();
            mPortAmbientScale.hide();
        }
        else
        {
            mPortAmbientSeparator.show();
            mPortAmbientScale.show();
        }
    }

    // diffuse

    if (mPortMode.isItemNew(1))
    {
        if (mPortMode.getValue(1) == 0)
        {
            mPortDiffuseSeparator.hide();
            mPortDiffuseScale.hide();
        }
        else
        {
            mPortDiffuseSeparator.show();
            mPortDiffuseScale.show();
        }
    }

    // specular

    if (mPortMode.isItemNew(2))
    {
        if (mPortMode.getValue(2) == 0)
        {
            mPortSpecularSeparator.hide();
            mPortSpecularScale.hide();
            mPortShininess.hide();
        }
        else
        {
            mPortSpecularSeparator.show();
            mPortSpecularScale.show();
            mPortShininess.show();
        }
    }
}

