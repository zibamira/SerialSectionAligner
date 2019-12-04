#include <stdio.h>

#include <hxlineviewer/HxLineRaycast.h>

#include <Inventor/SbLinear.h>
#include <Inventor/events/SoMouseButtonEvent.h>

#include <hxcore/HxObjectPool.h>

#include <hxlines/internal/HxLineSetInterface.h>
#include <hxspatialgraph/internal/HxSpatialGraphInterface.h>
#include <hxsurface/HxSurface.h>

#include <hxlineviewer/HxModuleMaterial.h>

#include <mclib/McPlane.h>
#include <mclib/McRot.h>

#include <algorithm>
#include <string>

namespace
{
    void addMaterial(HxSurface* surface, const QString& name, const SbColor& color, bool createPatch = false)
    {
        HxParamBundle* materials = surface->params->getMaterials();

        int id = materials->getNumberOfBundles();

        if (createPatch)
        {
            surface->patches.append(new HxSurface::Patch());
            surface->patches.last()->innerRegion = id;
            surface->patches.last()->outerRegion = 0;
        }

        int rgb[3];

        rgb[0] = int(color[0] * 255.0f);
        rgb[1] = int(color[1] * 255.0f);
        rgb[2] = int(color[2] * 255.0f);

        HxParamBundle* bundle = new HxParamBundle(name);
        bundle->insert(new HxParameter("id", id));
        bundle->insert(new HxParameter("Color", 3, rgb));
        materials->insert(bundle);

    }

    bool computeValues(HxSpatialGraphInterface* graph, GraphAttribute* attribute, HxSpatialGraph::Location location, McDArray< int >& values, McDArray< int >& patchIDs)
    {
        if (!graph || !attribute) return false;

        if (location == HxSpatialGraph::POINT) return false;

        EdgeVertexAttribute* att = (EdgeVertexAttribute*) attribute;

        int n = 0;

        if (location == HxSpatialGraph::VERTEX)
        {
            n = graph->getNumVertices();
        }
        else if (location == HxSpatialGraph::EDGE)
        {
            n = graph->getNumEdges();
        }

        patchIDs.resize(n);
        values.clear();

        // only integer 32 attributes are supported

        if (attribute->primType() != McPrimType::MC_INT32)
        {
            patchIDs.fill(0);
        }
        else
        {
            // collect different attributes and create a mapping

            for (int i = 0; i < n; ++i)
            {
                bool f = false;
                int  v = att->getIntDataAtIdx(i);

                for (int j = 0; j < values.size(); ++j)
                {
                    if (v == values[j])
                    {
                        patchIDs[i] = j;
                        f = true;
                        break;
                    }
                }

                if (!f)
                {
                    patchIDs[i] = values.size();
                    values.push(v);
                }
            }
        }

        return true;
    }

    void setTriangle(McDArray< int >& triangles, int id, int v1, int v2, int v3)
    {
        triangles[id * 3 + 0] = v1;
        triangles[id * 3 + 1] = v2;
        triangles[id * 3 + 2] = v3;
    }

    void
    createIkosaeder(McDArray< McVec3f >& points, McDArray< int >& triangles)
    {
        points.clear();
        points.resize(12);
        triangles.resize(20 * 3);

        double a = 4.0 / std::sqrt(2.0 * (5.0 + std::sqrt(5.0)));
        double b = (a / 4.0) * (1.0 + std::sqrt(5.0));
        double ah = a / 2.0;

        
        points[0].setValue(-ah, b, 0.0);
        points[1].setValue(ah, b, 0.0);
        points[2].setValue(-ah, -b, 0.0);
        points[3].setValue(ah, -b, 0.0);

        points[4].setValue(0.0, ah, -b);
        points[5].setValue(0.0, -ah, -b);
        points[6].setValue(0.0, ah, b);
        points[7].setValue(0.0, -ah, b);

        points[8].setValue(-b, 0.0, ah);
        points[9].setValue(-b, 0.0, -ah);
        points[10].setValue(b, 0.0, ah);
        points[11].setValue(b, 0.0, -ah);

        setTriangle(triangles, 0, 0, 1, 4);
        setTriangle(triangles, 1, 1, 11, 4);
        setTriangle(triangles, 2, 0, 4, 9);
        setTriangle(triangles, 3, 4, 11, 5);
        setTriangle(triangles, 4, 4, 5, 9);
        setTriangle(triangles, 5, 11, 3, 5);
        setTriangle(triangles, 6, 5, 2, 9);
        setTriangle(triangles, 7, 5, 3, 2);
        setTriangle(triangles, 8, 1, 10, 11);
        setTriangle(triangles, 9, 11, 10, 3);
        setTriangle(triangles, 10, 0, 9, 8);
        setTriangle(triangles, 11, 8, 9, 2);
        setTriangle(triangles, 12, 1, 0, 6);
        setTriangle(triangles, 13, 1, 6, 10);
        setTriangle(triangles, 14, 0, 8, 6);
        setTriangle(triangles, 15, 6, 7, 10);
        setTriangle(triangles, 16, 8, 7, 6);
        setTriangle(triangles, 17, 7, 3, 10);
        setTriangle(triangles, 18, 3, 7, 2);
        setTriangle(triangles, 19, 2, 7, 8);

        for (int i = 0; i < 12; ++i)
        {
            points[i].normalize();
        }
    }

    void createSphere(McDArray< McVec3f >& points, McDArray< int >& triangles, int detail)
    {
        McDArray< McVec3f > pointsIko;
        McDArray< int >     trianglesIko;

        createIkosaeder(pointsIko, trianglesIko);

        int startItt = 2 * detail + 1;
        int tIndex = 0;
        int numTri = trianglesIko.size() / 3;

        triangles.resize(3 * (numTri * (detail + 1) * (detail + 1)));
        points.resize(triangles.size());

        for (int i = 0; i < numTri; ++i)
        {
            McVec3f v1 = pointsIko[trianglesIko[i * 3 + 0]];
            McVec3f v2 = pointsIko[trianglesIko[i * 3 + 1]];
            McVec3f v3 = pointsIko[trianglesIko[i * 3 + 2]];

            McVec3f x = (1.0 / (float)(detail + 1)) * (v2 - v1);
            McVec3f y = (1.0 / (float)(detail + 1)) * (v3 - v1);

            int sI = startItt;

            for (int k = 0; k < detail + 1; ++k)
            {
                McVec3f vN1 = v1 + ((float)k * y);
                McVec3f vN2 = vN1;
                McVec3f vN3 = vN1;

                for (int j = sI; j > 0; --j)
                {
                    if (j % 2 == 1)
                    {
                        vN2 = vN1 + x;
                        vN3 = vN1 + y;

                        points[tIndex * 3 + 0] = vN1;
                        points[tIndex * 3 + 1] = vN2;
                        points[tIndex * 3 + 2] = vN3;

                        setTriangle(triangles, tIndex, tIndex * 3, tIndex * 3 + 1, tIndex * 3 + 2);

                        tIndex++;
                    }
                    else
                    {
                        vN1 = vN2;
                        vN2 = vN1 + y;

                        points[tIndex * 3 + 0] = vN1;
                        points[tIndex * 3 + 1] = vN2;
                        points[tIndex * 3 + 2] = vN3;

                        setTriangle(triangles, tIndex, tIndex * 3, tIndex * 3 + 1, tIndex * 3 + 2);

                        tIndex++;

                        vN2 = vN1;
                        vN2 = vN1;
                    }
                }
                sI -= 2;
            }
        }

        for (int i = 0; i < points.size(); ++i)
        {
            points[i].normalize();
        }

        // remove duplicate points

        McDArray< int > pointMap(points.size());

        for (int i = 0; i < points.size(); ++i)
        {
            pointMap[i] = -1;
        }

        // searches duplicated vertices and creates a vertex map

        McDArray< McVec3f > newPoints(0, points.size());

        for (int i = 0; i < points.size(); ++i)
        {
            for (int j = 0; j < newPoints.size(); ++j)
            {
                float d = (points[i] - newPoints[j]).length();

                if (d <= 0.0001)
                {
                    pointMap[i] = j;
                    break;
                }
            }

            if (pointMap[i] < 0)
            {
                pointMap[i] = newPoints.size();
                newPoints.push(points[i]);
            }
        }

        points = newPoints;

        // apply mapping to all groups

        for (int i = 0; i < triangles.size(); ++i)
        {
            triangles[i] = pointMap[triangles[i]];
        }
    }


    inline void
    invalid(float& x, float inv)
    {
        if (x != x || x > FLT_MAX)
            x = inv;
    }

    float
    getMaxCone(McVec3f diag, float vol, float maxV, float scale)
    {
        float max_edge = std::max(std::max(diag.x, diag.y), diag.z);
        float min_edge = 0.1 * max_edge;

        float v_B = scale * std::max(min_edge, diag.x) * std::max(min_edge, diag.y) * std::max(min_edge, diag.x);

        float s = powf(v_B / (M_PI * vol), 1.f / 2.f);

        invalid(s, 0.0);

        return s;
    }

    float
    getMaxSphere(McVec3f diag, float vol, float maxV, float scale)
    {
        float max_edge = std::max(std::max(diag.x, diag.y), diag.z);
        float min_edge = 0.1 * max_edge;

        float v_B = scale * std::max(min_edge, diag.x) * std::max(min_edge, diag.y) * std::max(min_edge, diag.x);

        float s = powf(v_B / (M_PI * vol), 1.f / 3.f);

        invalid(s, 0.0);

        return s;
    }

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

    McVec3f
    getTangent(const McDArray< McVec3f >& points, int id)
    {
        int next = std::min(int(points.size() - 1), int(id + 1));
        int prev = std::max(int(0),                 int(id - 1));

        McVec3f t =  points[next] - points[prev];

        t.normalize();

        return t;
    }

    void
    writeSphere(HxSurface& surface, const McDArray< McVec3f >& points, const McDArray< int >& triangles, int patch, int& vID, McVec3f position,float radius)
    {

        for (int i = 0; i < points.size(); ++i)
        {
            McVec3f p = position + (radius * points[i]);

            surface.points().push(p);
        }

        for (int i = 0; i < triangles.size() / 3; ++i)
        {
            Surface::Triangle triangle;

            triangle.points[0] = triangles[i * 3 + 0] +  vID;
            triangle.points[1] = triangles[i * 3 + 1] +  vID;
            triangle.points[2] = triangles[i * 3 + 2] +  vID;

            surface.appendTriangle(patch, triangle);
        }

        vID += points.size();
    }
}

HX_INIT_CLASS(HxLineRaycast, HxModule)

HxLineRaycast::~HxLineRaycast(void)
{
}

void
HxLineRaycast::compute()
{
    if (mRaycaster == NULL)
        return;

    if (mSpatialGraphInterface == NULL && mLineSetInterface == NULL)
        return;

    bool adaptEndingRadius = mAdaptLines && mPortEndingRadiusData.getValue(0);
    bool adaptEndingColor = mAdaptLines && mPortEndingColorStyle.getValue(0);
    bool adaptNodeColor = mAdaptLines && mPortNodeColorStyle.getValue(0);

    // change port display
    if (mPortDisplay.isNew())
    {
        mRaycaster->set((bool)mPortDisplay.getValue(0), SoLineRaycast::SEGMENT);
        mRaycaster->set((bool)mPortDisplay.getValue(1), SoLineRaycast::ENDING);
        mRaycaster->set((bool)mPortDisplay.getValue(2), SoLineRaycast::NODE);
    }

    // change port line radius data
    if (mPortLineRadiusData.isNew())
    {
        int data = mPortLineRadiusData.getValue(0);

        mRaycaster->setRadiusData(data - 1, mMaxDataSegments[data], SoLineRaycast::SEGMENT);

        updateRadiusScalePort(mPortLineRadiusScale, mRadiusMinMaxSegments[data].y);

        if (adaptEndingRadius)
        {
            mRaycaster->setRadiusData(data - 1, mMaxDataSegments[data], SoLineRaycast::ENDING);
        }
    }

    // change port line radius scale
    if (mPortLineRadiusScale.isNew())
    {
        mRaycaster->setRadiusScale(mPortLineRadiusScale.getValue(), SoLineRaycast::SEGMENT);

        if (adaptEndingRadius)
        {
            mRaycaster->setRadiusScale(mPortLineRadiusScale.getValue(), SoLineRaycast::ENDING);
        }
    }

    // change port line color style
    if (mPortLineColorStyle.isNew() ||
        mPortLineColorMap.isNew() ||
        mPortLineColorConstant.isNew())
    {
        int att = mPortLineColorStyle.getValue(0);

        mRaycaster->setColorData(att, SoLineRaycast::SEGMENT);

        // constant
        if (att == 0)
        {
            McColor c;
            mPortLineColorConstant.getColor(c);
            float rgb[3] = { c.r, c.g, c.b };
            mRaycaster->setColorMap(0, SoLineRaycast::SEGMENT, rgb);
        }
        // label
        else if (mPortLineColorStyle.getValue(1) && mPortLineColorStyle.getSensitivity(1))
            mRaycaster->setColorMap(0, SoLineRaycast::SEGMENT);
        // other
        else
            mRaycaster->setColorMap(&mPortLineColorMap, SoLineRaycast::SEGMENT);

        if (adaptEndingColor)
            mRaycaster->cloneColor(SoLineRaycast::SEGMENT, SoLineRaycast::ENDING);
        if (adaptNodeColor)
            mRaycaster->cloneColor(SoLineRaycast::SEGMENT, SoLineRaycast::NODE);
    }

    // change port line options
    if (mPortLineOptions.isNew())
    {
        mRaycaster->setNormalInterpolation((float)mPortLineOptions.getValue(0) - 0.5);
        mRaycaster->setSegmentAutoSpheres((bool)mPortLineOptions.getValue(1));
        mRaycaster->touch();
    }

    // change port line bend angle
    if (mPortLineBendAngle.isNew())
    {
        mRaycaster->setSegmentSphereAngle(mPortLineBendAngle.getValue());
    }

    // change port ending radius data
    if (mPortEndingRadiusData.isNew())
    {
        if (adaptEndingRadius)
        {
            int data = mPortLineRadiusData.getValue(0);

            mRaycaster->setRadiusScale(mPortLineRadiusScale.getValue(), SoLineRaycast::ENDING);
            mRaycaster->setRadiusData(data - 1, mMaxDataSegments[data], SoLineRaycast::ENDING);
        }
        else
        {
            int data = mPortEndingRadiusData.getValue(1);

            updateRadiusScalePort(mPortEndingRadiusScale, mRadiusMinMaxEnding[data].y);

            mRaycaster->setRadiusScale(mPortEndingRadiusScale.getValue(), SoLineRaycast::ENDING);
            mRaycaster->setRadiusData(data - 1, mMaxDataEnding[data], SoLineRaycast::ENDING);
        }
    }

    // change port ending radius scale
    if (mPortEndingRadiusScale.isNew() && !adaptEndingRadius)
    {
        mRaycaster->setRadiusScale(mPortEndingRadiusScale.getValue(), SoLineRaycast::ENDING);
    }

    // change port ending color
    if (mPortEndingColorStyle.isNew() ||
        mPortEndingColorConstant.isNew() ||
        mPortEndingColorMap.isNew())
    {
        int att = mPortEndingColorStyle.getValue(1);

        if (adaptEndingColor)
        {
            mRaycaster->cloneColor(SoLineRaycast::SEGMENT, SoLineRaycast::ENDING);
        }
        else
        {
            mRaycaster->setColorData(att, SoLineRaycast::ENDING);

            // constant
            if (att == 0)
            {
                McColor c;
                mPortEndingColorConstant.getColor(c);
                float rgb[3] = { c.r, c.g, c.b };
                mRaycaster->setColorMap(0, SoLineRaycast::ENDING, rgb);
            }
            // label
            else if (mPortEndingColorStyle.getValue(2) && mPortEndingColorStyle.getSensitivity(2))
                mRaycaster->setColorMap(0, SoLineRaycast::ENDING);
            // other
            else
                mRaycaster->setColorMap(&mPortEndingColorMap, SoLineRaycast::ENDING);
        }
    }

    // change port node radius data
    if (mPortNodeRadiusData.isNew())
    {
        int data = mPortNodeRadiusData.getValue(0);

        mRaycaster->setRadiusData(data - 1, mMaxDataNodes[data], SoLineRaycast::NODE);

        updateRadiusScalePort(mPortNodeRadiusScale, mRadiusMinMaxNodes[data].y);
    }

    // change port node radius scale
    if (mPortNodeRadiusScale.isNew())
    {
        mRaycaster->setRadiusScale(mPortNodeRadiusScale.getValue(), SoLineRaycast::NODE);
    }

    // change port node color
    if (mPortNodeColorStyle.isNew() ||
        mPortNodeColorMap.isNew() ||
        mPortNodeColorConstant.isNew())
    {
        int att = mPortNodeColorStyle.getValue(1);

        if (adaptNodeColor)
        {
            mRaycaster->cloneColor(SoLineRaycast::SEGMENT, SoLineRaycast::NODE);
        }
        else
        {
            mRaycaster->setColorData(att, SoLineRaycast::NODE);

            // constant
            if (att == 0)
            {
                McColor c;
                mPortNodeColorConstant.getColor(c);
                float rgb[3] = { c.r, c.g, c.b };
                mRaycaster->setColorMap(0, SoLineRaycast::NODE, rgb);
            }
            // label
            else if (mPortNodeColorStyle.getValue(2) && mPortNodeColorStyle.getSensitivity(2))
                mRaycaster->setColorMap(0, SoLineRaycast::NODE);
            // other
            else
                mRaycaster->setColorMap(&mPortNodeColorMap, SoLineRaycast::NODE);
        }
    }

    // set glow on/off
    if (mPortSetting.isItemNew(1))
    {
        mRaycaster->setGlow((bool)mPortSetting.getValue(1));
    }

    // set glow function
    if (mPortGlowFunc.isNew())
    {
        mRaycaster->setGlowFunc(mPortGlowFunc.getValue());
    }

    // set glow value
    if (mPortGlowValue.isNew())
    {
        mRaycaster->setGlowValue(mPortGlowValue.getValue());
    }

    // exports the current view as obj file
    if (mPortExport.isNew())
    {
        exportSurface();
    }

    // update selected edges
    updateSelectionPorts();

    showGeom(mRootNode);
}

HxLineRaycast::HxLineRaycast(void)
    : HxModule(HxLineSetInterface::getClassTypeId())
    , mConnMaterial(this, "material", tr("Material"), HxModuleMaterial::getClassTypeId())
    , mPortDisplay(this, "display", tr("Display"), 3)
    , mPortLineRadiusData(this, "lineRadius", tr("lineRadius"))
    , mPortLineRadiusScale(this, "lineRadiusScale", tr("lineRadiusScale"))
    , mPortLineColorStyle(this, "lineColor", tr("lineColor"))
    , mPortLineColorConstant(this, "lineConstColor", tr("lineConstColor"), 1)
    , mPortLineColorMap(this, "lineColormap", tr("lineColormap"))
    , mPortLineOptions(this, "lineOptions", tr("lineOptions"))
    , mPortLineBendAngle(this, "lineBendAngle", tr("lineBendAngle"))
    , mPortEndingRadiusData(this, "endingRadius", tr("endingRadius"))
    , mPortEndingRadiusScale(this, "endingRadiusScale", tr("endingRadiusScale"))
    , mPortEndingColorStyle(this, "endingColor", tr("endingColor"))
    , mPortEndingColorConstant(this, "endingConstColor", tr("endingConstColor"), 1)
    , mPortEndingColorMap(this, "endingColormap", tr("endingColormap"))
    , mPortNodeRadiusData(this, "nodeRadius", tr("nodeRadius"))
    , mPortNodeRadiusScale(this, "nodeRadiusScale", tr("nodeRadiusScale"))
    , mPortNodeColorStyle(this, "nodeColor", tr("nodeColor"))
    , mPortNodeColorConstant(this, "nodeConstColor", tr("nodeConstColor"), 1)
    , mPortNodeColorMap(this, "nodeColormap", tr("nodeColormap"))
    , mPortSetting(this, "options", tr("Options"), 2)
    , mPortGlowFunc(this, "glowFunc", tr("Glow Function"), 2)
    , mPortGlowValue(this, "glow", tr("Glow"))
    , mPortExportDetail(this, "exportDetail", tr("Export Detail"))
    , mPortExport(this, "exportSurface", tr("Export Surface"), 1)
    , mPortEdgeSelection(this, "edgeSelection", tr("Selected Edges"))
    , mAdaptLines(true)
    , mEmpty(true)
    , mLineSetInterface(NULL)
    , mSpatialGraphInterface(NULL)
    , mRangeSetModule(this)
{
    portData.addType(HxSpatialGraphInterface::getClassTypeId());

    // display settings
    mPortDisplay.setValue(0, 1);
    mPortDisplay.setValue(1, 1);
    mPortDisplay.setValue(2, 0);

    // set line color port settings
    mPortLineColorConstant.setColor(McColor(1.0, 1.0, 1.0));

    // set line option port settings
    mPortLineOptions.insertCheckBox(0, "smooth normals");
    mPortLineOptions.setValue(0, 1);
    mPortLineOptions.insertCheckBox(1, "smooth bends");
    mPortLineOptions.setValue(1, 1);

    // set line bend settings
    mPortLineBendAngle.setMinMax(0.0, M_PI);
    mPortLineBendAngle.setValue(2.0);

    // set ending color settings
    mPortEndingColorConstant.setColor(McColor(1.0, 1.0, 1.0));

    // set node color settings
    mPortNodeColorConstant.setColor(McColor(1.0, 1.0, 1.0));

    // set settings
    mPortSetting.setLabel(0, "auto scale");
    mPortSetting.setValue(0, 1);
    mPortSetting.setLabel(1, "glow");
    mPortSetting.setValue(1, 0);

    // glow
    mPortGlowFunc.setValue(0);
    mPortGlowFunc.setLabel(0, "additive");
    mPortGlowFunc.setLabel(1, "subtractive");

    mPortGlowValue.setMinMax(0.0, 5.0);
    mPortGlowValue.setValue(2.0);

    // export
    mPortExportDetail.setMinMax(1, 5);
    mPortExportDetail.setValue(3);
    mPortExport.setLabel(0, "export");

    // selection
    mPortEdgeSelection.setVisible(false);

    // specific state
    mLineSetDataValues = 0;


    McHandle<SoEventCallback> eventCB = new SoEventCallback;
    eventCB->addEventCallback(SoMouseButtonEvent::getClassTypeId(), mouseClickCB, this);

    mRootNode = new SoSeparator();
    mRaycaster = new SoLineRaycast();

    mRootNode->addChild(mRaycaster);
    mRootNode->addChild(eventCB);
}

void
HxLineRaycast::computeLineSetProperties()
{
    int numPoints = mLineSetInterface->getNumPoints();
    int numLines = mLineSetInterface->getNumLines();
    int numDatas = mLineSetInterface->getNumDataValues();
    McVec3f* coords = mLineSetInterface->getCoords();

    if (numPoints == 0 && numLines == 0)
        mEmpty = true;

    mRadiusMinMaxSegments.resize(numDatas + 1);
    mRadiusMinMaxEnding.resize(numDatas + 1);
    mRadiusMinMaxNodes.resize(numDatas + 1);
    mMaxDataSegments.resize(numDatas + 1);
    mMaxDataEnding.resize(numDatas + 1);
    mMaxDataNodes.resize(numDatas + 1);

    mCorner1 = McVec3f(FLT_MAX, FLT_MAX, FLT_MAX);
    mCorner2 = McVec3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    // get bounding box
    for (int i = 0; i < numPoints; ++i)
    {
        if (coords[i].x < mCorner1.x)
            mCorner1.x = coords[i].x;
        if (coords[i].x > mCorner2.x)
            mCorner2.x = coords[i].x;
        if (coords[i].y < mCorner1.y)
            mCorner1.y = coords[i].y;
        if (coords[i].y > mCorner2.y)
            mCorner2.y = coords[i].y;
        if (coords[i].z < mCorner1.z)
            mCorner1.z = coords[i].z;
        if (coords[i].z > mCorner2.z)
            mCorner2.z = coords[i].z;
    }

    McVec3f diag = mCorner2 - mCorner1;

    // get max data and set radius min max

    for (int j = 0; j < numDatas + 1; ++j)
    {
        float vol = 0.0;
        int numLines = mLineSetInterface->getNumLines();

        mMaxDataSegments[j] = 0.0;

        for (int i = 0; i < numLines; ++i)
        {
            int nP = mLineSetInterface->getLineLength(i);

            if (nP < 2)
                continue;

            // get the sum of all segment volumes
            for (int k = 0; k < nP - 1; ++k)
            {
                float r0 = 1.0;
                float r1 = 1.0;

                if (j > 0)
                    r0 = std::abs(mLineSetInterface->getData(i, k, j - 1));
                if (j > 0)
                    r1 = std::abs(mLineSetInterface->getData(i, k + 1, j - 1));

                McVec3f p0 = mLineSetInterface->getPoint(i, k);
                McVec3f p1 = mLineSetInterface->getPoint(i, k + 1);
                McVec3f d = p0 - p1;
                float h = d.length();
                float r_max = std::max(r0, r1);
                float r_min = std::min(r0, r1);

                invalid(h, 0.0);

                // volume cylinder
                if (std::abs(r0 - r1) <= 0.00001)
                {
                    vol += r_max * r_max * h;
                }
                // volume truncated pyramid
                else
                {
                    float l = (h * r_max) / std::abs(r1 - r0);

                    vol += (1.0 / 3.0) * (r_max * r_max * l - r_min * r_min * (l - h));
                }

                mMaxDataSegments[j] = std::max(mMaxDataSegments[j], r_max);
            }
        }

        mRadiusMinMaxSegments[j].x = 0.0;
        mRadiusMinMaxSegments[j].y = getMaxCone(diag, vol, mMaxDataSegments[j], 0.2);

        float* data = 0;
        float vol_N = 0.0;

        if (j > 0)
            data = mLineSetInterface->getData(j - 1);

        mMaxDataNodes[j] = 0.0;

        for (int i = 0; i < numPoints; ++i)
        {
            float r = 1.0;

            if (j > 0)
                r = std::abs(data[i]);

            invalid(r, 0.0);

            vol_N += 4.0 / 3.0 * r * r * r;

            mMaxDataNodes[j] = std::max(r, mMaxDataNodes[j]);
        }
        mMaxDataEnding[j] = mMaxDataNodes[j];

        mRadiusMinMaxNodes[j].x = 0.0;
        mRadiusMinMaxNodes[j].y = getMaxSphere(diag, vol_N, mMaxDataNodes[j], 0.5);
        mRadiusMinMaxEnding[j].x = 0.0;
        mRadiusMinMaxEnding[j].y = mRadiusMinMaxNodes[j].y * 1.5;
        ;
    }
}

void
HxLineRaycast::exportSurface()
{
    if (!mSpatialGraphInterface) return;

    // create surface

    HxSurface* surf = HxSurface::createInstance();

    addMaterial(surf, "Exterior", SbColor(1.0f, 1.0f, 1.0f));

    int pID = 0;
    int vID = 0;

    // export vertices

    if (mPortDisplay.getValue(1))
    {
        McDArray< int > patches(mSpatialGraphInterface->getNumVertices());

        // setup colors

        if (mPortEndingColorStyle.getValue(1) == 0)
        {
            SbColor color(1.0f, 1.0f, 1.0f);
            mPortEndingColorConstant.getColor((McColor&) color, 0);
            addMaterial(surf, "vertices", color, true);
            patches.fill(0);
        }
        else
        {
            EdgeVertexAttribute* colAtt = (EdgeVertexAttribute*) mEndingColorAtt[mPortEndingColorStyle.getValue(1) - 1].mAttribute;

            // floating point attributes are colored constant.

            if (colAtt->primType() != McPrimType::MC_INT32)
            {
                addMaterial(surf, "vertices", SbColor(1.0f, 1.0f, 1.0f), true);
                patches.fill(0);
            }
            else
            {
                McDArray< int > values(0, 255);

                computeValues(mSpatialGraphInterface, colAtt, HxSpatialGraph::VERTEX, values, patches);

                // add for each attribute a material

                for (int i = 0; i < values.size(); ++i)
                {
                    QString labelName = "vertex-material-" + QString::number(values[i]);
                    SbColor col(1.0f, 1.0f, 1.0f);

                    // label names
                    if (mSpatialGraphInterface->hasLabel(colAtt))
                    {
                        if (values[i] == 0)
                        {
                            labelName = QString("none");
                        }
                        else
                        {
                            McString name;
                            mSpatialGraphInterface->getLabelName(colAtt, values[i], name);
                            labelName = QString::fromLocal8Bit(name.getString());
                        }
                    }

                    // use label color or colormap

                    if (mSpatialGraphInterface->hasLabel(colAtt) &&
                        mPortEndingColorStyle.getValue(2) != 0)
                    {
                        mSpatialGraphInterface->getLabelColor(colAtt, values[i], col);
                    }
                    else
                    {
                        col = mPortEndingColorMap.getColor(float(values[i]));
                    }

                    addMaterial(surf, labelName, col, true);
                }
            }
        }

        // create sphere by ikosaeder subdivision

        McDArray< McVec3f > spherePoints;
        McDArray< int > sphereTriangles;

        createSphere(spherePoints, sphereTriangles, mPortExportDetail.getValue() - 1);

        // create one sphere per node

        EdgeVertexAttribute* radAtt = 0;

        if (mPortEndingRadiusData.getValue(0) > 0)
        {
            radAtt = (EdgeVertexAttribute*) mEndingRadiusAtt[mPortEndingRadiusData.getValue(0) - 1].mAttribute;
        }

        for (int i = 0; i < mSpatialGraphInterface->getNumVertices(); ++i)
        {
            McVec3f p = mSpatialGraphInterface->getVertexCoords(i);
            float   r = 1.0f;
            int     c = patches[i] + pID;

            if (radAtt)
            {
                if      (radAtt->primType() == McPrimType::MC_FLOAT) r = radAtt->getFloatDataAtIdx(i);
                else if (radAtt->primType() == McPrimType::MC_INT32) r = (float) radAtt->getIntDataAtIdx(i);
            }

            writeSphere(*surf, spherePoints, sphereTriangles, c, vID, p, r * mPortEndingRadiusScale.getValue());
        }
    }

    // write edges

    pID = surf->patches.size();

    if (mPortDisplay.getValue(0))
    {
        McDArray< int > patches (mSpatialGraphInterface->getNumEdges());

        int ringDetail = 5 * int(std::pow(1.51, double(mPortExportDetail.getValue() - 1)) + 0.5);

        if (mPortLineColorStyle.getValue(0) == 0 || // constant
            mPortLineColorStyle.getValue(0) - 1 >= mSegmentColorAtt.size()) // point attribute
        {
            SbColor col(1.0f, 1.0f, 1.0f);
            mPortLineColorConstant.getColor((McColor&) col, 0);
            addMaterial(surf, "edges", col, true);
            patches.fill(0);
        }
        else
        {
            EdgeVertexAttribute* colAtt = (EdgeVertexAttribute*) mSegmentColorAtt[mPortLineColorStyle.getValue(0) - 1].mAttribute;

            // floating point attributes are colored constant.

            if (colAtt->primType() != McPrimType::MC_INT32)
            {
                addMaterial(surf, "edges", SbColor(1.0f, 1.0f, 1.0f), true);
                patches.fill(0);
            }
            else
            {
                McDArray< int > values(0, 255);

                computeValues(mSpatialGraphInterface, colAtt, HxSpatialGraph::EDGE, values, patches);

                // add for each attribute a material

                for (int i = 0; i < values.size(); ++i)
                {
                    QString labelName = "edge-material-" + QString::number(values[i]);
                    SbColor col(1.0f, 1.0f, 1.0f);

                    if (mSpatialGraphInterface->hasLabel(colAtt))
                    {
                        if (values[i] == 0)
                        {
                            labelName = QString("none");
                        }
                        else
                        {
                            McString name;

                            mSpatialGraphInterface->getLabelName(colAtt, values[i], name);

                            labelName = QString::fromLocal8Bit(name.getString());
                        }
                    }

                    // use label color or colormap

                    if (mSpatialGraphInterface->hasLabel(colAtt) &&
                        mPortLineColorStyle.getValue(1) != 0)
                    {
                        mSpatialGraphInterface->getLabelColor(colAtt, values[i], col);
                    }
                    else
                    {
                        col = mPortLineColorMap.getColor(float(values[i]));
                    }

                    addMaterial(surf, labelName, col, true);
                }
            }
        }

        // detect radius

        GraphAttribute* radAtt = 0;
        bool            onPoints = false;
        
        if (mPortLineRadiusData.getValue(0) > 0)
        {
            int att = mPortLineRadiusData.getValue(0) - 1;

            if (att < mSegmentRadiusAtt.size())
                radAtt = mSegmentRadiusAtt[att].mAttribute;
            else
            {
                radAtt = mNodeRadiusAtt[att - mSegmentRadiusAtt.size()].mAttribute;
                onPoints = true;
            }
        }


        for (int i = 0; i < mSpatialGraphInterface->getNumEdges(); ++i)
        {
            // create vertex rings

            McVec3f pd(1.1f, 0.9f, 0.3f);
            float r = 1.0;

            if (radAtt && !onPoints)
            {
                if (radAtt->primType() == McPrimType::MC_INT32)
                    r = (float) ((EdgeVertexAttribute*) radAtt)->getIntDataAtIdx(i);
                else if (radAtt->primType() == McPrimType::MC_FLOAT)
                    r = ((EdgeVertexAttribute*) radAtt)->getFloatDataAtIdx(i);
            }

            for (int j = 0; j < mSpatialGraphInterface->getNumEdgePoints(i); ++j)
            {
                if (radAtt && onPoints)
                {
                    if (radAtt->primType() == McPrimType::MC_INT32)
                        r = (float) ((PointAttribute*)radAtt)->getIntDataAtPoint(i, j);
                    else if (radAtt->primType() == McPrimType::MC_FLOAT)
                        r = ((PointAttribute*)radAtt)->getFloatDataAtPoint(i, j);
                }

                McPlane t(getTangent(mSpatialGraphInterface->getEdgePoints(i), j));
                McVec3f p = mSpatialGraphInterface->getEdgePoint(i, j);
                McVec3f d = t.project(pd);

                d.normalize();
                pd = d;

                float alpha = (2.0f * M_PI) / float(ringDetail);

                McRotation rot(t.getNormal(), alpha);

                for (int k = 0; k < ringDetail; ++k)
                {
                    McVec3f v = p + d * r * mPortLineRadiusScale.getValue();

                    surf->points().push(v);

                    McVec3f nd;

                    rot.multVec(d, nd);

                    d = nd;
                    d.normalize();
                }
            }

            // create triangle strip cylinders

            for (int j = 1; j < mSpatialGraphInterface->getNumEdgePoints(i); ++j)
            {
                for (int k = 0; k < ringDetail; ++k)
                {
                    int v1 = vID + k;
                    int v2 = vID + (k + 1) % ringDetail;
                    int v3 = vID + ringDetail + k;
                    int v4 = vID + ringDetail + (k + 1) % ringDetail;

                    Surface::Triangle t1;
                    Surface::Triangle t2;

                    t1.points[0] = v1;
                    t1.points[1] = v4;
                    t1.points[2] = v3;

                    t2.points[0] = v1;
                    t2.points[1] = v2;
                    t2.points[2] = v4;

                    int c = patches[i] + pID;

                    surf->appendTriangle(c, t1);
                    surf->appendTriangle(c, t2);
                }
                vID += ringDetail;
            }
            vID += ringDetail;
        }
    }

    // push into pool

    surf->setLabel(QString("Spatialgraph-Surface"));

    theObjectPool->addObject(surf);
}

const McDArray<int>&
HxLineRaycast::getSelectedEdges() const
{
    return mRaycaster->getSelection(SoLineRaycast::SEGMENT);
}

McDArray<McVec2i>
HxLineRaycast::getSelectedPoints() const
{
    return mRaycaster->getSelectedNodes();
}

const McDArray<int>&
HxLineRaycast::getSelectedVertices() const
{
    return mRaycaster->getSelection(SoLineRaycast::ENDING);
}

void
HxLineRaycast::setSelectedEdges(const McDArray<int>& selected)
{
    mRaycaster->setSelection(selected, SoLineRaycast::SEGMENT);
}

void
HxLineRaycast::setSelectedVertices(const McDArray<int>& selected)
{
    mRaycaster->setSelection(selected, SoLineRaycast::ENDING);
}

void
HxLineRaycast::clearSelection()
{
    mRaycaster->clearSelection();
}

void
HxLineRaycast::computeSpatialGraphProperties()
{
    int numPoints = 0;
    int numVertices = mSpatialGraphInterface->getNumVertices();
    int numEdges = mSpatialGraphInterface->getNumEdges();

    mCorner1 = McVec3f(FLT_MAX, FLT_MAX, FLT_MAX);
    mCorner2 = McVec3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    // get bounding box
    for (int i = 0; i < numEdges; ++i)
    {
        int np = 0;
        const McVec3f* coords = mSpatialGraphInterface->getEdgePoints(i, np);

        for (int j = 0; j < np; ++j)
        {
            if (coords[j].x < mCorner1.x)
                mCorner1.x = coords[j].x;
            if (coords[j].x > mCorner2.x)
                mCorner2.x = coords[j].x;
            if (coords[j].y < mCorner1.y)
                mCorner1.y = coords[j].y;
            if (coords[j].y > mCorner2.y)
                mCorner2.y = coords[j].y;
            if (coords[j].z < mCorner1.z)
                mCorner1.z = coords[j].z;
            if (coords[j].z > mCorner2.z)
                mCorner2.z = coords[j].z;

            numPoints++;
        }
    }

    for (int i = 0; i < numVertices; ++i)
    {
        McVec3f coord = mSpatialGraphInterface->getVertexCoords(i);

        if (coord.x < mCorner1.x)
            mCorner1.x = coord.x;
        if (coord.x > mCorner2.x)
            mCorner2.x = coord.x;
        if (coord.y < mCorner1.y)
            mCorner1.y = coord.y;
        if (coord.y > mCorner2.y)
            mCorner2.y = coord.y;
        if (coord.z < mCorner1.z)
            mCorner1.z = coord.z;
        if (coord.z > mCorner2.z)
            mCorner2.z = coord.z;
    }

    // get max data and set radius min max

    McVec3f diag = mCorner2 - mCorner1;

    int numVertexAtt = mEndingRadiusAtt.size();
    int numSegAtt = mSegmentRadiusAtt.size();
    int numNoAtt = mNodeRadiusAtt.size();
    int numSegNoAtt = numSegAtt + numNoAtt + 1;

    mRadiusMinMaxSegments.resize(numSegNoAtt);
    mRadiusMinMaxEnding.resize(numVertexAtt + 1);
    mRadiusMinMaxNodes.resize(numSegNoAtt);
    mMaxDataSegments.resize(numSegNoAtt);
    mMaxDataEnding.resize(numVertexAtt + 1);
    mMaxDataNodes.resize(numSegNoAtt);

    // edge att
    for (int j = 0; j < numSegAtt; ++j)
    {
        const EdgeVertexAttribute* att = (EdgeVertexAttribute*)mSegmentRadiusAtt[j].mAttribute;

        float v = 0.0;
        mMaxDataSegments[j + 1] = 0.0;

        for (int i = 0; i < numEdges; ++i)
        {
            float r = std::abs(getFloat(att, i));
            v += mSpatialGraphInterface->getNumEdgePoints(i) * (4.0 / 3.0) * r * r * r;
            mMaxDataSegments[j + 1] = std::max(r, mMaxDataSegments[j + 1]);
        }

        mRadiusMinMaxSegments[j + 1].x = 0.0;
        mRadiusMinMaxSegments[j + 1].y = getMaxSphere(diag, v, mMaxDataSegments[j + 1], 0.25);
    }

    // point att
    for (int j = 0; j < numNoAtt; ++j)
    {
        const PointAttribute* att = (PointAttribute*)mNodeRadiusAtt[j].mAttribute;

        int i_e = j + numSegAtt + 1;
        float v = 0.0;

        mMaxDataSegments[i_e] = 0.0;

        for (int i = 0; i < numEdges; ++i)
        {
            int np = mSpatialGraphInterface->getNumEdgePoints(i);

            for (int k = 0; k < np; ++k)
            {
                float r = std::abs(getFloat(att, i, k));
                v += (4.0 / 3.0) * r * r * r;
                mMaxDataSegments[i_e] = std::max(mMaxDataSegments[i_e], r);
            }
        }

        mRadiusMinMaxSegments[i_e].x = 0.0;
        mRadiusMinMaxSegments[i_e].y = getMaxSphere(diag, v, mMaxDataSegments[i_e], 0.25);
    }

    // vert att
    for (int j = 0; j < numVertexAtt; ++j)
    {
        const EdgeVertexAttribute* att = (EdgeVertexAttribute*)mEndingRadiusAtt[j].mAttribute;

        float v = 0.0;
        mMaxDataEnding[j + 1] = 0.0;

        for (int i = 0; i < numVertices; ++i)
        {
            float r = std::abs(getFloat(att, i));
            v += 4.0 / 3.0 * r * r * r;
            mMaxDataEnding[j + 1] = std::max(mMaxDataEnding[j + 1], r);
        }

        mRadiusMinMaxEnding[j + 1].x = 0.0;
        mRadiusMinMaxEnding[j + 1].y = getMaxSphere(diag, v, mMaxDataEnding[j + 1], 0.10);
    }

    mMaxDataSegments[0] = 1.0;
    mRadiusMinMaxSegments[0].x = 0.0;
    mRadiusMinMaxSegments[0].y = getMaxSphere(diag, numPoints * (4.0 / 3.0), 1.0, 0.25);

    mMaxDataEnding[0] = 1.0;
    mRadiusMinMaxEnding[0].x = 0.0;
    mRadiusMinMaxEnding[0].y = getMaxSphere(diag, numVertices * (4.0 / 3.0), 1.0, 0.10);

    for (int i = 0; i < numSegNoAtt; ++i)
    {
        mMaxDataNodes[i] = mMaxDataSegments[i];
        mRadiusMinMaxNodes[i] = mRadiusMinMaxSegments[i];
    }
}

void
HxLineRaycast::mouseClick(SoEventCallback* eventCB)
{
    mRaycaster->mouseClick(eventCB);
}

int
HxLineRaycast::parse(Tcl_Interp* interpreter, int argc, char** argv)
{
    HxModule::parse(interpreter, argc, argv);

    // load selected vertices

    const std::string names[] = {"setSelectedEdges", "setSelectedVertices", "setSelectedPoints"};
    
    for (int i = 0; i < 3; ++i)
    {
        if (argc >= 3 && names[i] == std::string(argv[1]))
        {
            int numElems = 0;

            sscanf(argv[2], "%d", &numElems);

            McDArray<int> elems(numElems);

            for (int j = 0; j < numElems; ++j)
            {
                sscanf(argv[j + 3], "%d", &elems[j]);
            }
            mRaycaster->setSelection(elems, (SoLineRaycast::Target)i);
        }
    }

    mRaycaster->touchSelection();

    return TCL_OK;
}

void
HxLineRaycast::savePorts(FILE* fp)
{
    // save default ports

    HxModule::savePorts(fp);

    // save selections

    for (int i = 0; i < 3; ++i)
    {
        std::string name;
        if      (i == SoLineRaycast::SEGMENT) name = "setSelectedEdges";
        else if (i == SoLineRaycast::ENDING)  name = "setSelectedVertices";
        else if (i == SoLineRaycast::NODE)    name = "setSelectedPoints";
        else continue;

        const McDArray<int>& elems = mRaycaster->getSelection((SoLineRaycast::Target)i);

        if (elems.size() == 0)
            continue;

        fprintf(fp, "\"%s\" %s %d", getName(), name.c_str(), (int)elems.size());

        for (int i = 0; i < elems.size(); ++i)
            fprintf(fp, " %d", elems[i]);
        fprintf(fp, "\n");
    }

}

void
HxLineRaycast::update()
{
    // new input
    if (portData.isNew())
    {
        mEmpty = false;

        initPorts();

        mLineSetInterface = hxconnection_cast<HxLineSetInterface>(portData);
        mSpatialGraphInterface = hxconnection_cast<HxSpatialGraphInterface>(portData);

        if (mLineSetInterface)
        {
            computeLineSetProperties();
            mRaycaster->setLineSetInterface(mLineSetInterface);
        }
        if (mSpatialGraphInterface)
        {
            computeSpatialGraphProperties();

            mRaycaster->setSpatialGraphInterface(mSpatialGraphInterface);
            mRaycaster->setAttributes(mSegmentRadiusAtt, mSegmentColorAtt, SoLineRaycast::SEGMENT);
            mRaycaster->setAttributes(mEndingRadiusAtt, mEndingColorAtt, SoLineRaycast::ENDING);
            mRaycaster->setAttributes(mNodeRadiusAtt, mNodeColorAtt, SoLineRaycast::NODE);
        }

        if (!mLineSetInterface && !mSpatialGraphInterface)
        {
            mEmpty = true;
        }
        else
        {
            McVec2f minMax1 = mRadiusMinMaxSegments[mPortLineRadiusData.getValue(0)];
            McVec2f minMax2 = mRadiusMinMaxEnding[mPortEndingRadiusData.getValue(1)];
            McVec2f minMax3 = mRadiusMinMaxNodes[mPortNodeRadiusData.getValue(0)];

            updateRadiusScalePort(mPortLineRadiusScale, minMax1.y);
            updateRadiusScalePort(mPortEndingRadiusScale, minMax2.y);
            updateRadiusScalePort(mPortNodeRadiusScale, minMax3.y);

            mRaycaster->setBoundingBox(mCorner1, mCorner2);
        }

        mPortDisplay.touch();
    }

    // care about material
    if (mConnMaterial.isNew())
    {
        HxModuleMaterial* mod = hxconnection_cast<HxModuleMaterial>(mConnMaterial);
        if (mod)
            mRaycaster->setMaterial(mod->getMaterial());
    }

    // care about display port
    if (mPortDisplay.isNew() || portData.isNew(HxData::NEW_SOURCE) || portData.isNew(HxData::NEW_DATA))
    {
        if (mPortDisplay.getValue(0))
        {
            if (mPortLineOptions.getValue(1))
                mPortLineBendAngle.show();
            mPortLineColorStyle.show();
            mPortLineOptions.show();
            mPortLineRadiusData.show();
            mPortLineRadiusScale.show();
            mAdaptLines = true;
            mPortEndingRadiusData.setSensitivity(0, 1);
            mPortEndingColorStyle.setSensitivity(0, 1);
            mPortNodeColorStyle.setSensitivity(0, 1);
            mPortLineColorStyle.touch();
            mPortEndingColorStyle.touch();
            mPortEndingRadiusData.touch();
            mPortNodeColorStyle.touch();
        }
        else
        {
            mPortLineBendAngle.hide();
            mPortLineColorMap.hide();
            mPortLineColorConstant.hide();
            mPortLineColorStyle.hide();
            mPortLineOptions.hide();
            mPortLineRadiusData.hide();
            mPortLineRadiusScale.hide();
            mAdaptLines = false;
            mPortEndingRadiusData.setSensitivity(0, 0);
            mPortEndingColorStyle.setSensitivity(0, 0);
            mPortNodeColorStyle.setSensitivity(0, 0);
            mPortEndingColorStyle.touch();
            mPortEndingRadiusData.touch();
            mPortNodeColorStyle.touch();
        }

        if (mPortDisplay.getValue(1))
        {
            mPortEndingColorConstant.show();
            mPortEndingColorMap.show();
            mPortEndingColorStyle.show();
            mPortEndingRadiusData.show();
            mPortEndingRadiusScale.show();
            mPortEndingRadiusData.touch();
            mPortEndingColorStyle.touch();
        }
        else
        {
            mPortEndingColorConstant.hide();
            mPortEndingColorMap.hide();
            mPortEndingColorStyle.hide();
            mPortEndingRadiusData.hide();
            mPortEndingRadiusScale.hide();
        }

        if (mPortDisplay.getValue(2))
        {
            mPortNodeColorMap.show();
            mPortNodeColorStyle.show();
            mPortNodeRadiusData.show();
            mPortNodeRadiusScale.show();
            mPortNodeColorStyle.touch();
        }
        else
        {
            mPortNodeColorConstant.hide();
            mPortNodeColorMap.hide();
            mPortNodeColorStyle.hide();
            mPortNodeRadiusData.hide();
            mPortNodeRadiusScale.hide();
        }
    }

    // care about line options port
    if (mPortLineOptions.isNew())
    {
        if (mPortLineOptions.getValue(1))
            mPortLineBendAngle.show();
        else
            mPortLineBendAngle.hide();
    }

    // care about ending radius port
    if (mPortEndingRadiusData.isNew())
    {
        if (mPortEndingRadiusData.getValue(0) && mAdaptLines)
        {
            mPortEndingRadiusData.setSensitivity(1, 0);
            mPortEndingRadiusScale.hide();
        }
        else if (mPortDisplay.getValue(1))
        {
            mPortEndingRadiusData.setSensitivity(1, 1);
            mPortEndingRadiusScale.show();
        }
    }

    // care about glow on/off
    if (mPortSetting.isItemNew(1))
    {
        if (mPortSetting.getValue(1) == 0)
        {
            mPortGlowFunc.hide();
            mPortGlowValue.hide();
        }
        else
        {
            mPortGlowFunc.show();
            mPortGlowValue.show();
        }
    }

    updateLineColorPorts();
    updateEndingColorPorts();
    updateNodeColorPorts();
}

int
HxLineRaycast::HxRangeSetModuleImpl::getCurrentSet(const HxData* data, const HxPortColormap& colormap) const
{
    if (!data)
        return -1;

    HxSpatialGraphInterface* graph = mcinterface_cast<HxSpatialGraphInterface>(data);
    if (graph && graph == mOwner->mSpatialGraphInterface)
    {
        HxSpatialGraph::Location location;
        int set = -1;
        if (&colormap == &mOwner->mPortEndingColorMap)
        {
            location = HxSpatialGraph::VERTEX;
            set = mOwner->mPortEndingColorStyle.getValue(1) - 2;
        }
        else if (&colormap == &mOwner->mPortLineColorMap)
        {
            location = HxSpatialGraph::EDGE;
            set = mOwner->mPortLineColorStyle.getValue(1) - 2;
        }
        else if (&colormap == &mOwner->mPortNodeColorMap)
        {
            location = HxSpatialGraph::POINT;
            set = mOwner->mPortNodeColorStyle.getValue(1) - 2;
        }

        if (set >= 0)
            return HxSpatialGraph::encodeCurrentSetWithLocation(set, location);
    }

    HxLineSetInterface* lines = mcinterface_cast<HxLineSetInterface>(data);
    if (lines && lines == mOwner->mLineSetInterface)
    {
        int set = -1;
        if (&colormap == &mOwner->mPortEndingColorMap)
        {
            set = mOwner->mPortEndingColorStyle.getValue(1) - 2;
        }
        else if (&colormap == &mOwner->mPortLineColorMap)
        {
            set = mOwner->mPortLineColorStyle.getValue(1) - 2;
        }
        else if (&colormap == &mOwner->mPortNodeColorMap)
        {
            set = mOwner->mPortNodeColorStyle.getValue(1) - 2;
        }
        return set;
    }

    return -1;
}
