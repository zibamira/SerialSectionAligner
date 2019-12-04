#include <hxcore/HxResource.h>

#include <hxlines/internal/HxLineSetInterface.h>
#include <hxspatialgraph/internal/HxSpatialGraphInterface.h>

#include <mcgl/internal/mcgl.h>

#include <Inventor/SbBox.h>
#include <Inventor/SbColor.h>
#include <Inventor/SbLinear.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/elements/SoCacheElement.h>
#include <Inventor/elements/SoViewVolumeElement.h>
#include <Inventor/elements/SoLightElement.h>
#include <Inventor/elements/SoModelMatrixElement.h>
#include <Inventor/elements/SoClipPlaneElement.h>
#include <Inventor/elements/SoDepthBufferElement.h>
#include <Inventor/elements/SoPickStyleElement.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/details/SoPointDetail.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoSpotLight.h>
#include <Inventor/nodes/SoPointLight.h>

#include <hxlineviewer/SoLineRaycast.h>

#include <mclib/internal/McWatch.h>
#include <algorithm>
#include <vector>

#include <mclib/McRot.h>

namespace
{

    bool coneRayIntersection(const SbVec3f& coneStart,
                             const SbVec3f& coneEnd,
                             float radiusStart,
                             float radiusEnd,
                             const SbLine& ray,
                             float& intersection)
    {
        SbVec3f p1, p2;
        float r1, r2;

        if (radiusStart < radiusEnd)
        {
            p1 = coneStart;
            p2 = coneEnd;
            r1 = radiusStart;
            r2 = radiusEnd;
        }
        else
        {
            p1 = coneEnd;
            p2 = coneStart;
            r1 = radiusEnd;
            r2 = radiusStart;
        }

        float d = (p1 - p2).length();
        float a = (r2 - r1) / d;
        float b = r1;

        SbVec3f    coneAxis = p2 - p1;
        McRotation rotation((McVec3f)coneAxis, McVec3f(0.0, 0.0, 1.0));

        McVec3f rD;
        rotation.multVec((McVec3f)ray.getDirection(), rD);
        SbVec3f rS0 = ray.getPosition() - p1;
        McVec3f rS;
        rotation.multVec((McVec3f)rS0, rS);

        float x = rD.x * rD.x + rD.y * rD.y - a * a * rD.z * rD.z;
        float y = 2.0 * (rD.x * rS.x + rD.y * rS.y - a * a * rS.z * rD.z - a * b * rD.z);
        float z = rS.x * rS.x + rS.y * rS.y - a * a * rS.z * rS.z - b * b - 2.0 * a * b * rS.z;

        intersection = DBL_MAX;

        float t[2];

        // root copmutation
        float sgny = y < 0.0 ? -1.0 : 1.0;
        float w    = y * y - 4.0 * x * z;

        if (w < 0.0) return false;

        float q = -0.5 * (y + sgny * std::sqrt(w));

        t[0] = q / x;
        t[1] = z / q;

        bool hit = false;

        for (int i = 0; i < 2; ++i)
        {
            McVec3f pos = rS + t[i] * rD;

            if (pos.z < 0.0 || pos.z > d) continue;

            if (t[i] >= 0.0 && t[i] < intersection)
            {
                intersection = t[i];
                hit = true;
            }
        }

        return hit;
    }



    inline McVec3f
    getNoiseVec()
    {
        McVec3f res;

        res.x = 2.0 * (-0.5 + (double)rand() / (double)RAND_MAX);
        res.y = 2.0 * (-0.5 + (double)rand() / (double)RAND_MAX);
        res.z = 2.0 * (-0.5 + (double)rand() / (double)RAND_MAX);

        return res;
    }

    /// Returns attribute as float
    float
    getFloat(const EdgeVertexAttribute* att, int index)
    {
        float res = 0.0;
        if (att->primType() == McPrimType::MC_FLOAT)
            res = att->getFloatDataAtIdx(index);
        if (att->primType() == McPrimType::MC_INT32)
            res = (float)att->getIntDataAtIdx(index);

        if (res != res)
            res = 0.0;

        return res;
    }

    /// Returns attribute as float
    float
    getFloat(const PointAttribute* att, int edge, int point)
    {
        float res = 0.0;
        if (att->primType() == McPrimType::MC_FLOAT)
            res = att->getFloatDataAtPoint(edge, point);
        if (att->primType() == McPrimType::MC_INT32)
            res = (float)att->getIntDataAtPoint(edge, point);

        if (res != res)
            res = 0.0;

        return res;
    }

    inline float
    getColor(const McDArray<SoLineRaycast::TargetData>& targets, int data, bool perPoint, int edge, int point)
    {
        // color by edge attribute
        if (data >= 0 && !perPoint)
        {
            return getFloat((EdgeVertexAttribute*)targets[SoLineRaycast::SEGMENT].mAttColors[data].mAttribute, edge);
        }
        // color by point attribute
        if (data >= 0 && perPoint)
        {
            return getFloat((PointAttribute*)targets[SoLineRaycast::NODE].mAttColors[data].mAttribute, edge, point);
        }

        return 0.0;
    }

    inline float
    getRadius(const McDArray<SoLineRaycast::TargetData>& targets, int data, bool perPoint, int edge, int point)
    {
        // radius by edge attribute
        if (data >= 0 && !perPoint)
        {
            return std::abs(getFloat((EdgeVertexAttribute*)targets[SoLineRaycast::SEGMENT].mAttRadius[data].mAttribute, edge));
        }
        // radius by point attribute
        if (data >= 0 && perPoint)
        {
            return std::abs(getFloat((PointAttribute*)targets[SoLineRaycast::NODE].mAttRadius[data].mAttribute, edge, point));
        }

        return 1.0;
    }
}

SoLineRaycast::SpatialGraphAttribute::SpatialGraphAttribute(GraphAttribute* attribute)
    : mAttribute(attribute)
{
    if (mAttribute)
    {
        mName = QString::fromLatin1(mAttribute->getName());
    }
}

SO_NODE_SOURCE(SoLineRaycast);

SoLineRaycast::SoLineRaycast(void)
    : mRadiusMode(CONSTANT)
    , mCameraMode(PERSPECTIVE)
    , mParamSegmentNormalInterpolation(1.f)
    , mGlow(false)
    , mGlowFunc(0.0)
    , mGlowValue(2.0)
    , mLineSetInterface(NULL)
    , mSpatialGraphInterface(NULL)
    , mRefreshSelection(false)
    , mSegmentAutoSpheres(true)
    , mSegmentSphereAngle(2.0)
{
    SO_NODE_CONSTRUCTOR(SoLineRaycast);

    mMaterial = new SoMaterial;

    mTargets.resize(3);

    for (int i = 0; i < 3; ++i)
    {
        mTargets[i].mOn = true;
        mTargets[i].mColorData = -1;
        mTargets[i].mColorLabel = false;
        mTargets[i].mMaxData = 1.0;
        mTargets[i].mRadiusData = -1;
    }


    mTargets[SEGMENT].mShader.load("/share/shaders/hxlineviewer/Cylinder", 4, GL_POINTS, GL_TRIANGLE_STRIP);
    mSegmentSphereShader.load("/share/shaders/hxlineviewer/Sphere", 4, GL_POINTS, GL_TRIANGLE_STRIP);
    mTargets[ENDING].mShader.load("/share/shaders/hxlineviewer/Sphere", 4, GL_POINTS, GL_TRIANGLE_STRIP);
    mTargets[NODE].mShader.load("/share/shaders/hxlineviewer/Sphere", 4, GL_POINTS, GL_TRIANGLE_STRIP);
}

SoLineRaycast::~SoLineRaycast(void)
{
}

void
SoLineRaycast::initClass()
{
    SO_NODE_INIT_CLASS(SoLineRaycast, SoShape, "Shape");
}

void
SoLineRaycast::cloneColor(const SoLineRaycast::Target& src, const SoLineRaycast::Target& dst)
{
    int i = (int)src;
    int j = (int)dst;

    mTargets[j].mColorData = mTargets[i].mColorData;
    mTargets[j].mColorLabel = mTargets[i].mColorLabel;
    mTargets[j].mColormap = mTargets[i].mColormap;

    mRefresh = true;

    touch();
}

int
SoLineRaycast::getRadiusData(SoLineRaycast::Target target) const
{
    return mTargets[target].mRadiusData;
}

void
SoLineRaycast::GLRender(SoGLRenderAction* action)
{
    const SbViewVolume& vol = SoViewVolumeElement::get(action->getState());

    if (vol.getProjectionType() == SbViewVolume::PERSPECTIVE &&
        mCameraMode == ORTHOGRAPHIC)
    {
        mCameraMode = PERSPECTIVE;
        updateShader(SEGMENT);
        updateShader(ENDING);
        updateShader(NODE);
    }
    else if (vol.getProjectionType() == SbViewVolume::ORTHOGRAPHIC &&
             mCameraMode == PERSPECTIVE)
    {
        mCameraMode = ORTHOGRAPHIC;
        updateShader(SEGMENT);
        updateShader(ENDING);
        updateShader(NODE);
    }

    // retrieve lights from the state.
    float lights[4] = { 0, 0, 0, 0 };
    const SoNodeList& lightList = SoLightElement::getLights(action->getState());
    for (int i = 0; i < std::min(4, lightList.getLength()); i++)
    {
        const SoLight* light = (SoLight*)lightList.get(i);
        if (light->isOfType(SoPointLight::getClassTypeId()))
            lights[i] = 1;
        if (light->isOfType(SoSpotLight::getClassTypeId()))
            lights[i] = 2;
        if (light->isOfType(SoDirectionalLight::getClassTypeId()))
            lights[i] = 3;
    }

    // send materials.
    mMaterial->GLRender(action);

    const SbMatrix mat = SoModelMatrixElement::get(action->getState());

    SbVec3f translation;
    SbRotation rotation;
    SbVec3f scale;
    SbRotation s_rotation;

    mat.getTransform(translation, rotation, scale, s_rotation);

    float mvScale = scale[0];

    const SoClipPlaneElement* clipPlaneElement = SoClipPlaneElement::getInstance(action->getState());
    unsigned int numClipPlane = clipPlaneElement->getNum();

    if (mGlow)
    {
        SoDepthBufferElement::set(action->getState(), TRUE, FALSE, SoDepthBufferElement::LEQUAL, SbVec2f(0.f, 1.f));

        glPushAttrib(GL_COLOR_BUFFER_BIT);
        glEnable(GL_BLEND);

        if (mGlowFunc > 0.5)
            glBlendEquation(GL_FUNC_REVERSE_SUBTRACT);
        else
            glBlendEquation(GL_FUNC_ADD);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    }

    update();
    updateSelection();

    unsigned int contextID = action->getCacheContext();

    for (int i = 0; i < 3; ++i)
    {
        if (mTargets[i].mOn)
        {
            int size = mSegments.size();
            if (i == 1)
                size = mEnding.size();
            if (i == 2)
                size = mNodes.size();

            if (size == 0)
                continue;

            mTargets[i].mShader.enable(contextID);
            mTargets[i].mShader.setParameter1f(contextID, "radiusScale", mTargets[i].mRadiusScale);
            mTargets[i].mShader.setParameter1f(contextID, "scale", mvScale);
            mTargets[i].mShader.setParameter1i(contextID, "numClipPlane", numClipPlane);

            mTargets[i].mColormap.enable(contextID);

            int colormapTextureUnit = 0;
            mTargets[i].mShader.setParameter1i(contextID, "colormap", colormapTextureUnit);

            const float colormapRange[] = { mTargets[i].mColormap.getMin(),
                                            mTargets[i].mColormap.getMax() };
            const int colormapSize = mTargets[i].mColormap.getSize();

            mTargets[i].mShader.setParameter2f(contextID, "colormapRange", colormapRange);
            mTargets[i].mShader.setParameter1i(contextID, "colormapSize", colormapSize);
            mTargets[i].mShader.setParameter4f(contextID, "lights", lights);

            if (mGlow)
            {
                mTargets[i].mShader.setParameter1f(contextID, "glow", mGlowValue);
                mTargets[i].mShader.setParameter1f(contextID, "glowFunc", mGlowFunc);
            }

            if (i == (int)SEGMENT)
            {
                mTargets[SEGMENT].mShader.setParameter1f(contextID, "normalInterpolation", mParamSegmentNormalInterpolation);
            }

            bool vboOK = mTargets[i].mVBO.enableVertexAndTexCoordPointer(contextID);

            if (vboOK && mTargets[i].mVBO.getNumVertices() > 0)
            {
                safeRendering(mTargets[i].mVBO.getNumVertices());
            }

            mTargets[i].mVBO.disableVertexAndTexCoordPointer(contextID);
            mTargets[i].mColormap.disable();
            mTargets[i].mShader.disable();

            // render segment spheres

            if (i == (int)SEGMENT && !mGlow)
            {
                mSegmentSphereShader.enable(contextID);
                mSegmentSphereShader.setParameter1f(contextID, "radiusScale", mTargets[i].mRadiusScale);
                mSegmentSphereShader.setParameter1f(contextID, "scale", mvScale);
                mSegmentSphereShader.setParameter1i(contextID, "numClipPlane", numClipPlane);

                mSegmentSphereShader.setParameter4f(contextID, "lights", lights);

                mTargets[i].mColormap.enable(contextID);

                mSegmentSphereShader.setParameter1i(contextID, "colormap", 0);

                const float colormapRange[] = { mTargets[i].mColormap.getMin(),
                                                mTargets[i].mColormap.getMax() };
                const int colormapSize = mTargets[i].mColormap.getSize();

                mSegmentSphereShader.setParameter2f(contextID, "colormapRange", colormapRange);
                mSegmentSphereShader.setParameter1i(contextID, "colormapSize", colormapSize);

                bool vboOK = mSegmentSphereVBO.enableVertexAndTexCoordPointer(contextID);

                if (vboOK && mSegmentSphereVBO.getNumVertices() > 0)
                {
                    safeRendering(mSegmentSphereVBO.getNumVertices());
                }

                mSegmentSphereVBO.disableVertexAndTexCoordPointer(contextID);
                mTargets[i].mColormap.disable();
                mSegmentSphereShader.disable();
            }
        }
    }

    if (mGlow)
    {
        glPopAttrib();
    }

    SoCacheElement::invalidate(action->getState());
}

void
SoLineRaycast::generatePrimitives(SoAction* action)
{
}

void
SoLineRaycast::computeBBox(SoAction* action, SbBox3f& box, SbVec3f& center)
{
    float scale = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        if (mTargets[i].mOn)
            scale = std::max(scale, mTargets[i].mMaxData * mTargets[i].mRadiusScale);
    }

    box.setBounds(mBBoxMin - McVec3f(scale, scale, scale),
                  mBBoxMax + McVec3f(scale, scale, scale));
    center = box.getCenter();
}

void
SoLineRaycast::mouseClick(SoEventCallback* eventCB)
{
    const SoMouseButtonEvent* event = 0;

    if (!eventCB)
        return;
    if (!eventCB->getEvent())
        return;

    if (eventCB->getEvent()->isOfType(SoMouseButtonEvent::getClassTypeId()))
    {
        event = (SoMouseButtonEvent*)eventCB->getEvent();
    }

    if (!event)
        return;

    // left mouse button down

    if (SoMouseButtonEvent::isButtonPressEvent(event, SoMouseButtonEvent::BUTTON1))
    {
        const bool control = (bool)event->wasCtrlDown();
        if (!control)
            clearSelection();

        if (eventCB->getPickedPoint() &&
            eventCB->getPickedPoint()->getPath()->containsNode(this))
        {
            const SoDetail* pickDetail = eventCB->getPickedPoint()->getDetail();
            if (pickDetail && pickDetail->getTypeId() == SoPointDetail::getClassTypeId())
            {
                const SoPointDetail* pointDetail = (SoPointDetail*)pickDetail;
                int objectID = pointDetail->getCoordinateIndex();
                int targetID = pointDetail->getMaterialIndex();

                int selection = mTargets[targetID].mSelection.indexOf(objectID);

                if (selection >= 0)
                    mTargets[targetID].mSelection.remove(selection);
                else
                {
                    mTargets[targetID].mSelection.push(objectID);
                }
            }
        }
        mRefreshSelection = true;
        touch();
    }
}

McDArray<McVec2i>
SoLineRaycast::getSelectedNodes() const
{
    const McDArray<int>& selection = mTargets[NODE].mSelection;
    McDArray<McVec2i> result(selection.size());
    for(int i = 0; i < selection.size(); ++i)
        result[i].setValue(mNodes[selection[i]].mEdgeID, mNodes[selection[i]].mNodeID);
    return result;
}


const McDArray<int>&
SoLineRaycast::getSelection(Target target) const
{
    return mTargets[target].mSelection;
}

void
SoLineRaycast::setSelection(const McDArray<int>& selected, Target target)
{
    mTargets[target].mSelection.clear();
    mTargets[target].mSelection = selected;

    mRefreshSelection = true;
    touch();
    return;
}

void
SoLineRaycast::clearSelection()
{
    for (int i = 0; i < 3; ++i)
        mTargets[i].mSelection.clear();
}

void
SoLineRaycast::rayPick(SoRayPickAction* action)
{
    if (SoPickStyleElement::get(action->getState()) != SoPickStyleElement::SHAPE)
        return;

    action->setObjectSpace();
    action->setPickAll(false);

    const SbLine& ray = action->getLine();

    // pick ending
    float t_min = FLT_MAX;
    int o_min = -1;
    int type = -1;
    if (mTargets[ENDING].mOn)
    {
        for (int i = 0; i < mEnding.size(); ++i)
        {
            SbSphere sphere(mEnding[i].mPoint, mTargets[ENDING].mRadiusScale * mEnding[i].mRadius);

            SbVec3f isec;
            if (sphere.intersect(ray, isec))
            {
                float t = (ray.getPosition() - isec).length();
                if (t < t_min)
                {
                    t_min = t;
                    o_min = i;
                    type = ENDING;
                }
            }
        }
    }
    if (mTargets[NODE].mOn)
    {
        for (int i = 0; i < mNodes.size(); ++i)
        {
            SbSphere sphere(mNodes[i].mPoint, mTargets[NODE].mRadiusScale * mNodes[i].mRadius);

            SbVec3f isec;
            if (sphere.intersect(ray, isec))
            {
                float t = (ray.getPosition() - isec).length();
                if (t < t_min)
                {
                    t_min = t;
                    o_min = i;
                    type = NODE;
                }
            }
        }
    }
    if (mTargets[SEGMENT].mOn)
    {
        // find closest edge
        for (int i = 0; i < mSegments.size(); ++i)
        {
            float t;
            if (! coneRayIntersection(
                        mSegments[i].mPointStart, mSegments[i].mPointEnd,
                        mSegments[i].mRadiusStart * mTargets[SEGMENT].mRadiusScale,
                        mSegments[i].mRadiusEnd * mTargets[SEGMENT].mRadiusScale,
                        ray, t)) continue;
            if (t < t_min)
            {
                t_min = t;
                o_min = mSegments[i].mEdgeId;
                type = SEGMENT;
            }
        }
    }
    if (type == -1) return;

    SbVec3f isec = ray.getPosition() + t_min * ray.getDirection();
    SoPickedPoint* pp = action->addIntersection(isec);
    if (pp)
    {
        SoPointDetail* detail = new SoPointDetail();
        detail->setMaterialIndex(type);
        detail->setCoordinateIndex(o_min);
        pp->setDetail(detail, this);
    }
}

void
SoLineRaycast::safeRendering(int numVertices)
{
    McWatch watchLocal;
    McWatch watchGlobal;
    int calls = 0;
    int renderIndex = 0;
    int renderVertices = 1;
    double timeGlobal = 0.001;
    double timeLocal = 0.0;

    watchGlobal.start();
    do
    {
        calls++;

        watchLocal.start();
        glDrawArrays(GL_POINTS, renderIndex, renderVertices);
        timeLocal = std::max(0.001, (double)watchLocal.stop());

        renderIndex = renderIndex + renderVertices;
        renderVertices = (int)(0.02 / (timeLocal / (double)renderVertices));
        renderVertices = std::max(1, std::min(renderVertices, numVertices - renderIndex));

        timeGlobal = watchGlobal.getTime();
    } while (renderIndex < numVertices && timeGlobal < 2.0);
}

void
SoLineRaycast::setBoundingBox(const McVec3f& corner1, const McVec3f& corner2)
{
    mBBoxMin = corner1;
    mBBoxMax = corner2;
}

void
SoLineRaycast::setColorMapByLabel(SoLineRaycast::Target target)
{
    bool ev_att = true;
    int t_id = (int)target;
    int r_id = mTargets[t_id].mColorData;

    GraphAttribute* att = 0;

    // find current attribute of target
    if (t_id == 1)
    {
        att = mTargets[t_id].mAttColors[r_id - 1].mAttribute;
    }
    else if (t_id == 0 || t_id == 2)
    {
        if (r_id > mTargets[SEGMENT].mAttColors.size())
        {
            att = mTargets[NODE].mAttColors[r_id - mTargets[SEGMENT].mAttColors.size() - 1].mAttribute;
            ev_att = false;
        }
        else
        {
            att = mTargets[SEGMENT].mAttColors[r_id - 1].mAttribute;
        }
    }

    if (!att)
        return;

    // fill colormap
    int numData = att->size();
    float rgb[3];
    SbColor color;
    bool hit[256];

    for (int i = 0; i < 256; ++i)
        hit[i] = false;

    for (int i = 0; i < numData; ++i)
    {
        int label = -1;

        if (ev_att)
        {
            EdgeVertexAttribute* att_ev = (EdgeVertexAttribute*)att;
            label = att_ev->getIntDataAtIdx(i);
        }
        else
        {
            PointAttribute* att_p = (PointAttribute*)att;
            int* data = att_p->intDataAtPoint(0, 0);
            label = data[i];
        }

        if (label < 0 || label > 252 || hit[label])
            continue;

        if (mSpatialGraphInterface->getLabelColor(att, label, color))
        {
            color.getValue(rgb[0], rgb[1], rgb[2]);
            mTargets[t_id].mColormap.setColor(rgb, label + 1);
            hit[label] = true;
        }
    }

    rgb[0] = 0.1;
    rgb[1] = 0.1;
    rgb[2] = 0.1;

    mTargets[t_id].mColormap.setColor(rgb, 254);
    mTargets[t_id].mColormap.setMinMax(0.0, 253.0);
}

void
SoLineRaycast::set(bool on, SoLineRaycast::Target t)
{
    mTargets[(int)t].mOn = on;

    mRefresh = true;

    touch();
}

void
SoLineRaycast::setAttributes(McDArray<SpatialGraphAttribute>& radAtt,
                             McDArray<SpatialGraphAttribute>& colAtt,
                             SoLineRaycast::Target target)
{
    mTargets[(int)target].mAttColors = colAtt;
    mTargets[(int)target].mAttRadius = radAtt;
}

void
SoLineRaycast::setColorData(int data, SoLineRaycast::Target t)
{
    int t_id = (int)t;

    mTargets[t_id].mColorData = data;
    mRefresh = true;

    touch();
}

void
SoLineRaycast::setColorMap(HxPortColormap* colormap, const SoLineRaycast::Target& t, float* rgb)
{
    int t_id = (int)t;

    // set default min max
    mTargets[t_id].mColorLabel = false;
    mTargets[t_id].mColormap.setMinMax(0.0, 1.0);

    int data = mTargets[t_id].mColorData;

    // set default colormap
    if (data < 0)
    {
        mTargets[t_id].mColormap.fill(0.9, 0.9, 0.9);
    }

    // set constant color
    else if (data == 0 && rgb != 0)
    {
        mTargets[t_id].mColormap.fill(rgb);
    }

    // set label colormap (only spatial graph)
    else if (data > 0 && colormap == NULL)
    {
        mTargets[t_id].mColorLabel = true;
        setColorMapByLabel(t);
    }

    // set colormap
    else if (data > 0 && colormap != NULL)
    {
        mTargets[t_id].mColormap.fill(colormap, 256);
    }

    touch();
}

void
SoLineRaycast::setGlow(bool on)
{
    mGlow = on;

    updateShader(SEGMENT);
    updateShader(ENDING);
    updateShader(NODE);

    touch();
}

void
SoLineRaycast::setGlowFunc(int func)
{
    mGlowFunc = double(func);

    touch();
}

void
SoLineRaycast::setGlowValue(double value)
{
    mGlowValue = value;

    touch();
}

void
SoLineRaycast::setMaterial(HxMaterial material)
{
    mMaterial->ambientColor.setValue(material.mAmbient ? material.mAmbientScale : 0.2f,
                                     material.mAmbient ? material.mAmbientScale : 0.2f,
                                     material.mAmbient ? material.mAmbientScale : 0.2f);

    mMaterial->diffuseColor.setValue(material.mDiffuse ? material.mDiffuseScale / 2.f : 0.8f,
                                     material.mDiffuse ? material.mDiffuseScale / 2.f : 0.8f,
                                     material.mDiffuse ? material.mDiffuseScale / 2.f : 0.8f);

    mMaterial->specularColor.setValue(material.mSpecular ? material.mSpecularScale : 0.0f,
                                      material.mSpecular ? material.mSpecularScale : 0.0f,
                                      material.mSpecular ? material.mSpecularScale : 0.0f);

    mMaterial->shininess = material.mShininess / 70.f;

    touch();
}

void
SoLineRaycast::setRadiusData(int data, float maxData, SoLineRaycast::Target t)
{
    int t_id = (int)t;

    if (t == SEGMENT)
    {
        if ((mLineSetInterface && data > -1) ||
            (mSpatialGraphInterface && data >= mTargets[t_id].mAttRadius.size()))
        {
            mRadiusMode = INDIVIDUAL;
            updateShader(SEGMENT);
        }
        else if ((mLineSetInterface && data == -1) ||
                 (mSpatialGraphInterface && data < mTargets[t_id].mAttRadius.size()))
        {
            mRadiusMode = CONSTANT;
            updateShader(SEGMENT);
        }
    }

    mTargets[t_id].mRadiusData = data;
    mTargets[t_id].mMaxData = maxData;

    mRefresh = true;

    touch();
}

void
SoLineRaycast::setRadiusScale(float scale, SoLineRaycast::Target t)
{
    mTargets[(int)t].mRadiusScale = scale;

    touch();
}

void
SoLineRaycast::setLineSetInterface(HxLineSetInterface* lineSetInterface)
{
    if (lineSetInterface == NULL)
        return;

    mLineSetInterface = lineSetInterface;
    mSpatialGraphInterface = 0;

    mRefresh = true;
}

void
SoLineRaycast::setSegmentAutoSpheres(bool on)
{
    if (on == mSegmentAutoSpheres)
        return;

    mSegmentAutoSpheres = on;
    mRefresh = true;

    touch();
}

void
SoLineRaycast::setSegmentSphereAngle(double angle)
{
    mSegmentSphereAngle = angle;

    mRefresh = true;

    touch();
}

void
SoLineRaycast::setSpatialGraphInterface(HxSpatialGraphInterface* graph)
{
    if (graph == 0)
        return;

    mLineSetInterface = 0;
    mSpatialGraphInterface = graph;

    mRefresh = true;
}

void
SoLineRaycast::touchSelection()
{
    mRefreshSelection = true;
}

void
SoLineRaycast::setNormalInterpolation(float value)
{
    mParamSegmentNormalInterpolation = value;
    touch();
}

void
SoLineRaycast::update()
{
    if (!mRefresh)
        return;

    mRefreshSelection = true;

    if (mLineSetInterface != NULL)
    {
        if (mTargets[SEGMENT].mOn)
            updateSegmentsFromLineInterface();
        if (mTargets[ENDING].mOn)
            updateEndingFromLineInterface();
        if (mTargets[NODE].mOn)
            updateNodesFromLineInterface();
    }
    else if (mSpatialGraphInterface != NULL)
    {
        if (mTargets[SEGMENT].mOn)
            updateSegmentsFromSpatialGraph();
        if (mTargets[ENDING].mOn)
            updateEndingFromSpatialGraph();
        if (mTargets[NODE].mOn)
            updateNodesFromSpatialGraph();
    }
    mRefresh = false;
}

void
SoLineRaycast::updateSelection()
{
    if (!mRefreshSelection)
        return;

    // ending
    for (int i = 0; i < mEnding.size(); ++i)
        mEnding[i].mSelected = 0.f;

    for (int i = 0; i < mTargets[ENDING].mSelection.size(); ++i)
        mEnding[mTargets[ENDING].mSelection[i]].mSelected = 1.f;

    mTargets[ENDING].mVBO.setBuffer((int)mEnding.size(), (int)sizeof(VertexSphere), mEnding.dataPtr());

    // node
    for (int i = 0; i < mNodes.size(); ++i)
        mNodes[i].mSelected = 0.f;

    for (int i = 0; i < mTargets[NODE].mSelection.size(); ++i)
        mNodes[mTargets[NODE].mSelection[i]].mSelected = 1.f;

    mTargets[NODE].mVBO.setBuffer((int)mNodes.size(), (int)sizeof(VertexSphere), mNodes.dataPtr());

    // segments
    for (int i = 0; i < mSegments.size(); ++i)
    {
        mSegments[i].mSelected = 0.f;
        if (mTargets[SEGMENT].mSelection.indexOf(mSegments[i].mEdgeId) >= 0)
                mSegments[i].mSelected = 1.f;
    }

    mTargets[SEGMENT].mVBO.setBuffer((int)mSegments.size(), (int)sizeof(VertexSegment), mSegments.dataPtr());

    mRefreshSelection = false;
}

void
SoLineRaycast::updateEndingFromLineInterface()
{
    mEnding.clear();

    TargetData& tar = mTargets[ENDING];

    int numLines = mLineSetInterface->getNumLines();
    int colData = tar.mColorData - 1;
    int radData = tar.mRadiusData;

    for (int i = 0; i < numLines; ++i)
    {
        int i_e = mLineSetInterface->getLineLength(i) - 1;

        if (i_e < 0)
            continue;

        VertexSphere s;

        s.mPoint = mLineSetInterface->getPoint(i, 0);
        s.mColorValue = 0.0;
        s.mRadius = 1.0;

        if (colData >= 0)
            s.mColorValue = mLineSetInterface->getData(i, 0, colData);
        if (radData >= 0)
            s.mRadius = std::abs(mLineSetInterface->getData(i, 0, radData));

        mEnding.push_back(s);

        if (i_e > 0)
        {
            VertexSphere e;

            e.mPoint = mLineSetInterface->getPoint(i, i_e);
            e.mColorValue = 0.0;
            e.mRadius = 1.0;

            if (colData >= 0)
                e.mColorValue = mLineSetInterface->getData(i, i_e, colData);
            if (radData >= 0)
                e.mRadius = std::abs(mLineSetInterface->getData(i, i_e, radData));

            mEnding.push_back(e);
        }
    }

    tar.mVBO.setBuffer((int)mEnding.size(), (int)sizeof(VertexSphere), mEnding.dataPtr());
}

void
SoLineRaycast::updateEndingFromSpatialGraph()
{
    mEnding.clear();

    TargetData& tar = mTargets[ENDING];

    int numVertices = mSpatialGraphInterface->getNumVertices();
    int colData = tar.mColorData - 1;
    int radData = tar.mRadiusData;

    for (int i = 0; i < numVertices; ++i)
    {
        VertexSphere s;

        s.mPoint = mSpatialGraphInterface->getVertexCoords(i);
        s.mColorValue = 0.0;
        s.mRadius = 1.0;

        if (colData >= 0)
        {
            EdgeVertexAttribute* att = (EdgeVertexAttribute*)tar.mAttColors[colData].mAttribute;
            s.mColorValue = getFloat(att, i);
        }
        if (radData >= 0)
            s.mRadius = std::abs(getFloat((EdgeVertexAttribute*)tar.mAttRadius[radData].mAttribute, i));

        mEnding.push_back(s);
    }

    tar.mVBO.setBuffer((int)mEnding.size(), (int)sizeof(VertexSphere), mEnding.dataPtr());
}

void
SoLineRaycast::updateNodesFromLineInterface()
{
    mNodes.clear();

    TargetData& tar = mTargets[NODE];

    int colData = tar.mColorData - 1;
    int radData = tar.mRadiusData;
    int numLines = mLineSetInterface->getNumLines();


    for (int l = 0; l < numLines; ++l)
    {
        int numNodes = mLineSetInterface->getLineLength(l);
        for (int n = 0; n < numNodes; ++n)
        {
            VertexSphere s;

            int i = mLineSetInterface->getLineVertex(l, n);
            s.mPoint = mLineSetInterface->getCoords()[i];
            s.mColorValue = 0.0;
            s.mRadius = 1.0;
            s.mSelected = 0.0;
            s.mEdgeID = l;
            s.mNodeID = n;

            if (colData >= 0)
                s.mColorValue = mLineSetInterface->getData(colData)[i];
            if (radData >= 0)
                s.mRadius = std::abs(mLineSetInterface->getData(radData)[i]);

            mNodes.push_back(s);
        }
    }

    tar.mVBO.setBuffer((int)mNodes.size(), (int)sizeof(VertexSphere), mNodes.dataPtr());
}

void
SoLineRaycast::updateNodesFromSpatialGraph()
{
    mNodes.clear();

    TargetData& tar = mTargets[NODE];

    int numEdges = mSpatialGraphInterface->getNumEdges();
    int colData = tar.mColorData - 1;
    int radData = tar.mRadiusData;
    bool perPointR = false;
    bool perPointC = false;

    if (radData >= mTargets[SEGMENT].mAttRadius.size())
    {
        perPointR = true;
        radData = radData - mTargets[SEGMENT].mAttRadius.size();
    }
    if (colData >= mTargets[SEGMENT].mAttColors.size())
    {
        perPointC = true;
        colData = colData - mTargets[SEGMENT].mAttColors.size();
    }

    for (int i = 0; i < numEdges; ++i)
    {
        int numPoints = mSpatialGraphInterface->getNumEdgePoints(i);

        for (int j = 0; j < numPoints; ++j)
        {
            VertexSphere s;

            s.mPoint = mSpatialGraphInterface->getEdgePoint(i, j);
            s.mColorValue = getColor(mTargets, colData, perPointC, i, j);
            s.mRadius = getRadius(mTargets, radData, perPointR, i, j);
            s.mSelected = 0.0;
            s.mEdgeID = i;
            s.mNodeID = j;

            mNodes.push_back(s);
        }
    }

    tar.mVBO.setBuffer((int)mNodes.size(), (int)sizeof(VertexSphere), mNodes.dataPtr());
}

void
SoLineRaycast::updateSegmentsFromLineInterface()
{
    mSegmentSpheres.clear();
    mSegments.clear();

    float minData = FLT_MAX;
    int numLines = mLineSetInterface->getNumLines();
    int colData = mTargets[SEGMENT].mColorData - 1;
    int radData = mTargets[SEGMENT].mRadiusData;

    std::vector<int> spheres(0, 1000);

    // generate segments
    for (int i = 0; i < numLines; ++i)
    {
        int numPoints = mLineSetInterface->getLineLength(i);

        if (numPoints < 2)
            continue;

        int e = numPoints - 1;

        McVec3f p_s = 2.f * mLineSetInterface->getPoint(i, 0) - mLineSetInterface->getPoint(i, 1);
        McVec3f p_e = 2.f * mLineSetInterface->getPoint(i, e) - mLineSetInterface->getPoint(i, e - 1);

        float r_s = 1.0;
        float r_e = 1.0;

        if (radData >= 0)
        {
            r_s = std::abs(mLineSetInterface->getData(i, 0, radData));
            r_e = std::abs(mLineSetInterface->getData(i, e, radData));

            if (r_e < minData)
                minData = r_e;
        }

        for (int j = 0; j < numPoints - 1; ++j)
        {
            VertexSegment seg;

            seg.mPointPre = p_s;
            seg.mPointStart = mLineSetInterface->getPoint(i, j);
            seg.mPointEnd = mLineSetInterface->getPoint(i, j + 1);
            seg.mPointPost = p_e;

            seg.mRadiusPre = r_s;
            seg.mRadiusPost = r_e;
            seg.mRadiusStart = 1.0;
            seg.mRadiusEnd = 1.0;

            seg.mColorStart = 0.0;
            seg.mColorEnd = 0.0;

            if (radData >= 0)
            {
                seg.mRadiusStart = std::abs(mLineSetInterface->getData(i, j, radData));
                seg.mRadiusEnd = std::abs(mLineSetInterface->getData(i, j + 1, radData));

                if (seg.mRadiusStart < minData)
                    minData = seg.mRadiusStart;
            }

            // coloring
            if (colData >= 0)
            {
                seg.mColorStart = mLineSetInterface->getData(i, j, colData);
                seg.mColorEnd = mLineSetInterface->getData(i, j + 1, colData);
            }

            // set pre point
            if (j > 0)
            {
                seg.mPointPre = mLineSetInterface->getPoint(i, j - 1);
                if (radData >= 0)
                {
                    seg.mRadiusPre = std::abs(mLineSetInterface->getData(i, j - 1, radData));
                }
            }
            // set post point
            if (j < numPoints - 2)
            {
                seg.mPointPost = mLineSetInterface->getPoint(i, j + 2);
                if (radData >= 0)
                {
                    seg.mRadiusPost = std::abs(mLineSetInterface->getData(i, j + 2, radData));
                }
            }

            // automatic spheres
            if (mSegmentAutoSpheres)
            {
                McVec3f v_s = seg.mPointPre - seg.mPointStart;
                McVec3f v_e = seg.mPointPost - seg.mPointEnd;

                if (v_s.angle(seg.mPointEnd - seg.mPointStart) < mSegmentSphereAngle)
                {
                    seg.mPointPre = 2.f * seg.mPointStart - seg.mPointEnd;
                    seg.mRadiusPre = seg.mRadiusStart;
                }

                if (v_e.angle(seg.mPointStart - seg.mPointEnd) < mSegmentSphereAngle)
                {
                    seg.mPointPost = 2.f * seg.mPointEnd - seg.mPointStart;
                    seg.mRadiusPost = seg.mRadiusEnd;

                    if (j < numPoints - 2)
                        spheres.push_back(mSegments.size());
                }
            }

            mSegments.push_back(seg);
        }
    }

    // add noise - to do: another solution
    if (radData >= 0)
    {
        McVec3f n0 = getNoiseVec();
        McVec3f n1 = getNoiseVec();
        McVec3f n2 = getNoiseVec();
        McVec3f n3 = getNoiseVec();

        int numSegs = (int)mSegments.size();
        float scale = 0.001 * mTargets[SEGMENT].mRadiusScale;

        for (int i = 0; i < numSegs; ++i)
        {
            mSegments[i].mPointPre += minData * scale * n0;
            mSegments[i].mPointStart += minData * scale * n1;
            mSegments[i].mPointEnd += minData * scale * n2;
            mSegments[i].mPointPost += minData * scale * n3;

            n0 = n1;
            n1 = n2;
            n2 = n3;
            n3 = getNoiseVec();
        }
    }

    // add spheres
    int numSpheres = (int)spheres.size();

    mSegmentSpheres.resize(numSpheres);

    for (int i = 0; i < numSpheres; ++i)
    {
        mSegmentSpheres[i].mPoint = mSegments[spheres[i]].mPointEnd;
        mSegmentSpheres[i].mRadius = mSegments[spheres[i]].mRadiusEnd;
        mSegmentSpheres[i].mColorValue = mSegments[spheres[i]].mColorEnd;
    }

    mTargets[SEGMENT].mVBO.setBuffer((int)mSegments.size(), (int)sizeof(VertexSegment), mSegments.dataPtr());
    mSegmentSphereVBO.setBuffer((int)mSegmentSpheres.size(), (int)sizeof(VertexSphere), mSegmentSpheres.dataPtr());
}

void
SoLineRaycast::updateSegmentsFromSpatialGraph()
{
    mSegmentSpheres.clear();
    mSegments.clear();

    TargetData& tar = mTargets[SEGMENT];

    float minData = FLT_MAX;
    int numEdges = mSpatialGraphInterface->getNumEdges();
    int colData = tar.mColorData - 1;
    int radData = tar.mRadiusData;
    bool perPointR = false;
    bool perPointC = false;

    // edge or point attribute for radius
    if (radData >= tar.mAttRadius.size())
    {
        perPointR = true;
        radData = radData - tar.mAttRadius.size();
    }

    // edge or point attribute for colors
    if (colData >= tar.mAttColors.size())
    {
        perPointC = true;
        colData = colData - tar.mAttColors.size();
    }

    std::vector<int> spheres(0, 1000);

    // generate segments
    for (int i = 0; i < numEdges; ++i)
    {
        int numPoints = mSpatialGraphInterface->getNumEdgePoints(i);

        if (numPoints < 2)
            continue;

        int e = numPoints - 1;

        McVec3f p_s = 2.f * mSpatialGraphInterface->getEdgePoint(i, 0) - mSpatialGraphInterface->getEdgePoint(i, 1);
        McVec3f p_e = 2.f * mSpatialGraphInterface->getEdgePoint(i, e) - mSpatialGraphInterface->getEdgePoint(i, e - 1);

        float r_s = getRadius(mTargets, radData, perPointR, i, 0);
        float r_e = getRadius(mTargets, radData, perPointR, i, e);

        for (int j = 0; j < numPoints - 1; ++j)
        {
            VertexSegment seg;

            seg.mPointPre = p_s;
            seg.mPointStart = mSpatialGraphInterface->getEdgePoint(i, j);
            seg.mPointEnd = mSpatialGraphInterface->getEdgePoint(i, j + 1);
            seg.mPointPost = p_e;

            seg.mRadiusPre = r_s;
            seg.mRadiusPost = r_e;
            seg.mRadiusStart = getRadius(mTargets, radData, perPointR, i, j);
            seg.mRadiusEnd = getRadius(mTargets, radData, perPointR, i, j + 1);

            seg.mColorStart = getColor(mTargets, colData, perPointC, i, j);
            seg.mColorEnd = getColor(mTargets, colData, perPointC, i, j + 1);

            seg.mEdgeId = i;

            if (seg.mRadiusStart < minData)
                minData = seg.mRadiusStart;

            // set pre point
            if (j > 0)
            {
                seg.mPointPre = mSpatialGraphInterface->getEdgePoint(i, j - 1);
                seg.mRadiusPre = getRadius(mTargets, radData, perPointR, i, j - 1);
            }
            // set post point
            if (j < numPoints - 2)
            {
                seg.mPointPost = mSpatialGraphInterface->getEdgePoint(i, j + 2);
                seg.mRadiusPost = getRadius(mTargets, radData, perPointR, i, j + 2);
            }

            // automatic spheres
            if (mSegmentAutoSpheres)
            {
                McVec3f v_s = seg.mPointPre - seg.mPointStart;
                McVec3f v_e = seg.mPointPost - seg.mPointEnd;

                if (v_s.angle(seg.mPointEnd - seg.mPointStart) < mSegmentSphereAngle)
                {
                    seg.mPointPre = 2.f * seg.mPointStart - seg.mPointEnd;
                    seg.mRadiusPre = seg.mRadiusStart;
                }

                if (v_e.angle(seg.mPointStart - seg.mPointEnd) < mSegmentSphereAngle)
                {
                    seg.mPointPost = 2.f * seg.mPointEnd - seg.mPointStart;
                    seg.mRadiusPost = seg.mRadiusEnd;

                    if (j < numPoints - 2)
                        spheres.push_back(mSegments.size());
                }
            }

            mSegments.push_back(seg);
        }
    }

    // add noise - to do: another solution
    if (radData >= 0 && perPointR)
    {
        McVec3f n0 = getNoiseVec();
        McVec3f n1 = getNoiseVec();
        McVec3f n2 = getNoiseVec();
        McVec3f n3 = getNoiseVec();

        int numSegs = (int)mSegments.size();
        float scale = 0.001 * tar.mRadiusScale;

        for (int i = 0; i < numSegs; ++i)
        {
            mSegments[i].mPointPre += minData * scale * n0;
            mSegments[i].mPointStart += minData * scale * n1;
            mSegments[i].mPointEnd += minData * scale * n2;
            mSegments[i].mPointPost += minData * scale * n3;

            n0 = n1;
            n1 = n2;
            n2 = n3;
            n3 = getNoiseVec();
        }
    }

    // add spheres
    int numSpheres = (int)spheres.size();

    mSegmentSpheres.resize(numSpheres);

    for (int i = 0; i < numSpheres; ++i)
    {
        mSegmentSpheres[i].mPoint = mSegments[spheres[i]].mPointEnd;
        mSegmentSpheres[i].mRadius = mSegments[spheres[i]].mRadiusEnd;
        mSegmentSpheres[i].mColorValue = mSegments[spheres[i]].mColorEnd;
    }

    tar.mVBO.setBuffer((int)mSegments.size(), (int)sizeof(VertexSegment), mSegments.dataPtr());
    mSegmentSphereVBO.setBuffer((int)mSegmentSpheres.size(), (int)sizeof(VertexSphere), mSegmentSpheres.dataPtr());
}

void
SoLineRaycast::updateShader(Target target)
{
    int t = (int)target;

    McString shaderDir;

    if (target == SEGMENT)
    {
        McString shaderDir2;

        if (mGlow)
        {
            if (mCameraMode == PERSPECTIVE)
            {
                shaderDir = "/share/shaders/hxlineviewer/LineGlow";
                shaderDir2 = "/share/shaders/hxlineviewer/SphereGlow";
            }
            else
            {
                shaderDir = "/share/shaders/hxlineviewer/LineGlowOrtho";
                shaderDir2 = "/share/shaders/hxlineviewer/SphereGlowOrtho";
            }
        }
        else
        {
            if (mRadiusMode == INDIVIDUAL && mCameraMode == PERSPECTIVE)
            {
                shaderDir = "/share/shaders/hxlineviewer/Cone";
                shaderDir2 = "/share/shaders/hxlineviewer/Sphere";
            }
            else if (mRadiusMode == CONSTANT && mCameraMode == PERSPECTIVE)
            {
                shaderDir = "/share/shaders/hxlineviewer/Cylinder";
                shaderDir2 = "/share/shaders/hxlineviewer/Sphere";
            }
            else if (mRadiusMode == INDIVIDUAL && mCameraMode == ORTHOGRAPHIC)
            {
                shaderDir = "/share/shaders/hxlineviewer/ConeOrtho";
                shaderDir2 = "/share/shaders/hxlineviewer/SphereOrtho";
            }
            else if (mRadiusMode == CONSTANT && mCameraMode == ORTHOGRAPHIC)
            {
                shaderDir = "/share/shaders/hxlineviewer/CylinderOrtho";
                shaderDir2 = "/share/shaders/hxlineviewer/SphereOrtho";
            }
        }
        mSegmentSphereShader.load(shaderDir2.getString(), 4, GL_POINTS, GL_TRIANGLE_STRIP);
    }
    else
    {
        if (mGlow)
        {
            if (mCameraMode == PERSPECTIVE)
            {
                shaderDir = "/share/shaders/hxlineviewer/SphereGlow";
            }
            else
            {
                shaderDir = "/share/shaders/hxlineviewer/SphereGlowOrtho";
            }
        }
        else
        {
            if (mCameraMode == PERSPECTIVE)
            {
                shaderDir = "/share/shaders/hxlineviewer/Sphere";
            }
            else
            {
                shaderDir = "/share/shaders/hxlineviewer/SphereOrtho";
            }
        }
    }

    mTargets[t].mShader.load(shaderDir.getString(), 4, GL_POINTS, GL_TRIANGLE_STRIP);
}
