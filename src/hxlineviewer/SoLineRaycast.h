#pragma once

#include <hxlineviewer/api.h>

#include <mclib/McVec3.h>
#include <mclib/McDArray.h>

#include <Inventor/nodes/SoShape.h>
#include <Inventor/nodes/SoMaterial.h>

#include <hxglutils/HxGLColormap.h>
#include <hxglutils/HxGLShader.h>
#include <hxglutils/HxGLVertexBufferObject.h>
#include <hxlineviewer/HxMaterial.h>

class HxLineSetInterface;
class HxSpatialGraphInterface;
class GraphAttribute;
class HxPortColormap;
class SoRayPickAction;
class McVec2i;

class HXLINEVIEWER_API SoLineRaycast : public SoShape
{
    SO_NODE_HEADER(SoLineRaycast);

public:
    SoLineRaycast(void);
    virtual ~SoLineRaycast(void);

    class SpatialGraphAttribute
    {
    public:

        SpatialGraphAttribute()
            : mAttribute(0)
        {
        }

        SpatialGraphAttribute(GraphAttribute* attribute);

        GraphAttribute* mAttribute;
        QString         mName;
    };

    struct TargetData
    {
        McDArray<SpatialGraphAttribute> mAttColors;
        McDArray<SpatialGraphAttribute> mAttRadius;
        bool mOn;
        int mColorData;
        bool mColorLabel;
        HxGLColormap mColormap;
        float mMaxData;
        int mRadiusData;
        float mRadiusScale;
        McDArray<int> mSelection;
        HxGLShader mShader;
        HxGLVertexBufferObject mVBO;
    };

    enum Target
    {
        SEGMENT,
        ENDING,
        NODE
    };

    /// Removes the selected vertices from internal data structure.
    void clearSelection();

    /// Clones the color properties.
    void cloneColor(const Target& src, const Target& dst);

    /// initialize class
    static void initClass();

    /// returns the radius data
    int getRadiusData(Target target) const;

    /// Returns selected elements.
    const McDArray<int>& getSelection(Target target = ENDING) const;
    //
    /// Returns selected NODES as pairs (edge_id, point_id in edge)
    McDArray<McVec2i> getSelectedNodes() const;

    void mouseClick(SoEventCallback* eventCB);

    /// Sets target on/off.
    void set(bool on, Target t);

    /// Sets ending attributes.
    void setAttributes(McDArray<SpatialGraphAttribute>& radAtt, McDArray<SpatialGraphAttribute>& colAtt, Target target);

    /// Sets bounding box.
    void setBoundingBox(const McVec3f& corner1, const McVec3f& corner2);

    /// Sets color data for target.
    void setColorData(int data, Target t);

    /// Sets a color map for target.
    void setColorMap(HxPortColormap* colormap, const Target& t, float* rgb = 0);

    /// Sets haze on/off
    void setGlow(bool on = true);

    /// Sets glow func (0 = additive, 1 = subtractive)
    void setGlowFunc(int func);

    /// Sets haze value
    void setGlowValue(double value);

    /// Sets a line set.
    void setLineSetInterface(HxLineSetInterface* lineSetInterface);

    /// Sets the material
    void setMaterial(HxMaterial material);

    /// Sets radius data for ending.
    void setRadiusData(int data, float maxData, Target t);

    /// Sets radius scale for target.
    void setRadiusScale(float scale, Target t);

    /// Sets segment auto spheres on/off
    void setSegmentAutoSpheres(bool on);

    /// Sets the angle for auto spheres.
    void setSegmentSphereAngle(double angle);

    /// Set the selected elements without raycasting.
    void setSelection(const McDArray<int>& selected, Target target = ENDING);

    /// Sets a spatial graph.
    void setSpatialGraphInterface(HxSpatialGraphInterface* graph);

    /// Updates the graphical representation of the selection.
    void touchSelection();

    void setNormalInterpolation(float value);

protected:
    // render
    virtual void GLRender(SoGLRenderAction* action);

    // generates primitives (in our case nothing)
    virtual void generatePrimitives(SoAction* action);

    // computes the bounding box
    virtual void computeBBox(SoAction* action, SbBox3f& box, SbVec3f& center);

    // computes picking
    virtual void rayPick(SoRayPickAction* action);

private:
    void setColorMapByLabel(Target target);

    void safeRendering(int numVertices);

    void update();

    void updateSelection();

    void updateShader(Target target);

    void updateEndingFromLineInterface();

    void updateEndingFromSpatialGraph();

    void updateNodesFromLineInterface();

    void updateNodesFromSpatialGraph();

    void updateSegmentsFromLineInterface();

    void updateSegmentsFromSpatialGraph();

    enum RadiusMode
    {
        CONSTANT,
        INDIVIDUAL
    };
    RadiusMode mRadiusMode;

    enum CameraMode
    {
        PERSPECTIVE,
        ORTHOGRAPHIC
    };
    CameraMode mCameraMode;

    // rendering
    float mParamSegmentNormalInterpolation;

    McVec3f mBBoxMin;
    McVec3f mBBoxMax;
    bool mGlow;
    double mGlowFunc;
    double mGlowValue;
    HxLineSetInterface* mLineSetInterface;
    HxSpatialGraphInterface* mSpatialGraphInterface;

    McHandle<SoMaterial> mMaterial;

    bool mRefresh;
    bool mRefreshSelection;

    McDArray<TargetData> mTargets;

    struct VertexSphere // for Targets NODE and ENDING
    {
        VertexSphere()
            : mSelected(0.f)
        {
        }
        McVec3f mPoint;
        float mRadius;

        float mColorValue;
        float mSelected;
        int   mEdgeID; // only relevant for Target NODE
        int   mNodeID; // only relavant for Target NODE
    };
    McDArray<VertexSphere> mEnding;
    McDArray<VertexSphere> mNodes;

    struct VertexSegment
    {
        McVec3f mPointPre;
        float mRadiusPre;

        McVec3f mPointStart;
        float mRadiusStart;

        McVec3f mPointEnd;
        float mRadiusEnd;

        McVec3f mPointPost;
        float mRadiusPost;

        float mColorStart;
        float mColorEnd;

        int mEdgeId; 

        float mSelected;
    };
    McDArray<VertexSegment> mSegments;

    bool mSegmentAutoSpheres;
    float mSegmentSphereAngle;
    McDArray<VertexSphere> mSegmentSpheres;
    HxGLShader mSegmentSphereShader;
    HxGLVertexBufferObject mSegmentSphereVBO;
};
