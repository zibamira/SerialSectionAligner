#pragma once

#include <hxlineviewer/api.h>

#include <hxcore/HxModule.h>
#include <hxcore/HxConnection.h>
#include <hxcore/HxPortColorList.h>
#include <hxcore/HxPortButtonList.h>
#include <hxcore/HxPortFloatSlider.h>
#include <hxcore/HxPortIntSlider.h>
#include <hxcore/HxPortText.h>
#include <hxcore/HxPortToggleList.h>
#include <hxcore/HxPortGeneric.h>

#include <hxcolor/HxPortColormap.h>
#include <hxcolor/internal/HxRangeSet.h>

#include <mclib/McDArray.h>
#include <mclib/McHandle.h>
#include <mclib/McVec3.h>
#include <mclib/McVec2.h>

#include <Inventor/nodes/SoSeparator.h>

#include <hxlineviewer/SoLineRaycast.h>

class HxLineSetInterface;
class HxSpatialGraphInterface;

class HXLINEVIEWER_API HxLineRaycast : public HxModule
{
    HX_HEADER(HxLineRaycast);

public:

    /**
            Computation function.
        */
    virtual void compute();

    /**
            Returns the selected edges.
        */
    const McDArray<int>& getSelectedEdges() const;

    /**
            Returns the selected points as tuples (edge_id, point_id).
        */
    McDArray<McVec2i> getSelectedPoints() const;

    /**
            Returns the selected vertices.
        */
    const McDArray<int>& getSelectedVertices() const;

    /**
            Set the selected edges without ray casting.
        */
    void setSelectedEdges(const McDArray<int>& selected);

    /**
            Set the selected vertices without ray casting.
        */
    void setSelectedVertices(const McDArray<int>& selected);

    /**
            Removes the selected vertices from internal data structure.
        */
    void clearSelection();

    /**
            Loads the port settings and the selected
            elements.
        */
    virtual int parse(Tcl_Interp* interpreter, int argc, char** argv);

    /**
            Updates the user interface.
        */
    void update();

    /**
            Saves all ports and afterwards the selected
            elements.
        */
    virtual void savePorts(FILE* fp);

    HxConnection mConnMaterial;

    HxPortToggleList mPortDisplay;

    HxPortGeneric mPortLineRadiusData;
    HxPortFloatSlider mPortLineRadiusScale;
    HxPortGeneric mPortLineColorStyle;
    HxPortColorList mPortLineColorConstant;
    HxPortColormap mPortLineColorMap;
    HxPortGeneric mPortLineOptions;
    HxPortFloatSlider mPortLineBendAngle;

    HxPortGeneric mPortEndingRadiusData;
    HxPortFloatSlider mPortEndingRadiusScale;
    HxPortGeneric mPortEndingColorStyle;
    HxPortColorList mPortEndingColorConstant;
    HxPortColormap mPortEndingColorMap;

    HxPortGeneric mPortNodeRadiusData;
    HxPortFloatSlider mPortNodeRadiusScale;
    HxPortGeneric mPortNodeColorStyle;
    HxPortColorList mPortNodeColorConstant;
    HxPortColormap mPortNodeColorMap;

    HxPortToggleList mPortSetting;
    HxPortRadioBox mPortGlowFunc;
    HxPortFloatSlider mPortGlowValue;

    HxPortIntSlider mPortExportDetail;
    HxPortButtonList mPortExport;

    HxPortText mPortEdgeSelection;



private:

    typedef SoLineRaycast::SpatialGraphAttribute SpatialGraphAttribute;

    static void
    mouseClickCB(void* module, SoEventCallback* eventCallback)
    {
        ((HxLineRaycast*)module)->mouseClick(eventCallback);
    }

    class HxRangeSetModuleImpl : public HxRangeSetModule
    {
    public:
        HxRangeSetModuleImpl(HxLineRaycast* o)
            : HxRangeSetModule(o->getOwner(), McInterfaceOwner::STATIC_IFACE)
            , mOwner(o)
        {
        }

        /** Returns the current data set encoded as described at HxSpatialGraph::getRangeSet().
                    Returns -1 if there is no current data set selected. */
        virtual int getCurrentSet(const HxData* data, const HxPortColormap& colormap) const;

    private:
        HxLineRaycast* mOwner;
    };

    friend class HxRangeSetModuleImpl;

    void computeLineSetProperties();

    void computeSpatialGraphProperties();

    void deletePortContent();

    void exportSurface();

    void initPorts();

    void initForLineSetInterface(HxLineSetInterface* newLineSet);

    void initEndingPortsForSpatialGraph(HxSpatialGraphInterface* newSpatialGraph);

    void initSegmentAndPointPortsForSpatialGraph(HxSpatialGraphInterface* newSpatialGraph);

    void mouseClick(SoEventCallback* eventCB);

    void updateSelectionPorts();

    void updateEndingColorPorts();

    void updateLineColorPorts();

    void updateNodeColorPorts();

    void updateRadiusScalePort(HxPortFloatSlider& slider, float max);

    // oiv
    McHandle<SoLineRaycast> mRaycaster;
    McHandle<SoSeparator> mRootNode;

    // properties
    bool mAdaptLines;
    bool mEmpty;
    McVec3f mCorner1;
    McVec3f mCorner2;

    McDArray<McVec2f> mRadiusMinMaxSegments;
    McDArray<McVec2f> mRadiusMinMaxEnding;
    McDArray<McVec2f> mRadiusMinMaxNodes;
    McDArray<float> mMaxDataSegments;
    McDArray<float> mMaxDataEnding;
    McDArray<float> mMaxDataNodes;

    McDArray<SpatialGraphAttribute> mEndingRadiusAtt;
    McDArray<SpatialGraphAttribute> mEndingColorAtt;
    McDArray<SpatialGraphAttribute> mSegmentRadiusAtt;
    McDArray<SpatialGraphAttribute> mSegmentColorAtt;
    McDArray<SpatialGraphAttribute> mNodeRadiusAtt;
    McDArray<SpatialGraphAttribute> mNodeColorAtt;

    // some specific state
    int mLineSetDataValues;

    // pointer on data
    HxLineSetInterface* mLineSetInterface;
    HxSpatialGraphInterface* mSpatialGraphInterface;

    HxRangeSetModuleImpl mRangeSetModule;
};
