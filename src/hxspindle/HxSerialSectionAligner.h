#pragma once

#include <hxspindle/api.h>
#include <hxspindle/HxSerialSectionStack.h>

#include <hxfield/HxLattice3.h>
#include <hxspatialgraph/internal/HxSpatialGraph.h>

#include <hxalignmicrotubules/mtalign.h>
#include <hxcoremodules/internal/HxAnnotation.h>
#include <hxcore/HxConnection.h>
#include <hxcore/HxModule.h>
#include <hxcore/HxPortColorList.h>
#include <hxcore/HxPortFloatSlider.h>
#include <hxcore/HxPortGeneric.h>
#include <hxcore/HxPortIntSlider.h>
#include <hxcore/HxPortMultiMenu.h>
#include <hxcore/HxPortToggleList.h>
#include <hxcore/HxPortSeparator.h>
#include <hxcore/HxPortGeneric.h>

#include <hxcolor/internal/HxRangeSet.h>

#include <mclib/McDArray.h>
#include <mclib/McHandle.h>
#include <mclib/McVec3.h>
#include <mclib/McVec2.h>
#include <mclib/McBox3f.h>

#include <mcgl/internal/mcgl.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCallback.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/actions/SoAction.h>

#include <hxspindle/SoSerialSectionAligner.h>




class HXSPINDLE_API HxSerialSectionAligner : public HxModule
{

    HX_HEADER(HxSerialSectionAligner);

    private:


        static void mouseClickCB(void* module, SoEventCallback* eventCallback)
        {
            ((HxSerialSectionAligner*)module)->mouseClick(eventCallback);
        }


    // type definitions

    private:


        typedef HxSerialSectionStack::Landmarks        Landmarks;
        typedef HxSerialSectionStack::Matching         Matching;
        typedef HxSerialSectionStack::Section          Section;
        typedef HxSerialSectionStack::SectionInterface SectionInterface;

    // methods

    public:


        /**
            Computation function.
        */
        virtual void compute();


        /**
            Loads the port settings and the selected
            elements.
        */
        virtual int parse(Tcl_Interp* interpreter, int argc, char **argv);


        /**
            Saves all ports and afterwards the selected
            elements.
        */
        virtual void savePorts(FILE* fp);


    private:


        /**
            Computes the next check position in the check array.
        */
        void checkNext(int direction = 1);


        /**
            Checks positions with selection.
        */
        void checkPos(const McVec2i& selection);


        /**
            Computes a field which shows already confirmed areas.
        */
        void computeCheckField();


        /**
            Computes the matching in the following way. Computes first
            a CPD transformation based on the microtubule ends. If sigma
            is too large the CPD transformation will be rejected. Then,
            the automatic matching algorithm is applied and finally again
            the CPD transformation is computed, but only for the matched
            filaments.
        */
        void computeMatching();


        /**
            Computes automatic landmarks based on matched filament ends.
        */
        void computeMatchingLandmarks(HxSerialSectionStack::SettingType type);


        /**
            Computes automatic landmarks based on matched filament ends.
        */
        void computeMatchingLandmarksFast(HxSerialSectionStack::SettingType type);


        void mouseClick(SoEventCallback* eventCB);


        void updateAnnotation();


        void updateCheck();


        void updateCheckCurrent();


        void updateCheckLabel();


        void updateResolutionLabel(bool initialize = false);


    // Ports

    public :

        HxConnection                        connMatchingColors;

        HxPortRadioBox                      portMode;
        HxPortIntSlider                     portSectionInterface;
        HxPortFloatSlider                   portSliceTop;
        HxPortFloatSlider                   portSliceBottom;
        HxPortFloatSlider                   portFilamentScale;
        HxPortToggleList                    portSynchronizeSlices;
        HxPortToggleList                    portColorBlind;
        HxPortRadioBox                      portSliceQuality;
        HxPortRadioBox                      portSliceQualityInteraction;
        HxPortIntSlider                     portWarpingQuality;
        HxPortFloatSlider                   portLandmarkScale;
        HxPortToggleList                    portLandmarkShow;
        HxPortButtonList                    portLandmarkDelete;
        HxPortFloatSlider                   portMatchingRegion;
        HxPortGeneric                       portMatching;
        HxPortGeneric                       portMatchingCheck;
        HxPortFloatSlider                   port3DRadius;
        HxPortIntSlider                     portSectionSelection;
        HxPortGeneric                       portComputeStack;
        HxPortGeneric                       portComputeStackResolution;


    // Attributes

    private :

        // mouse

        bool                                mMouseLeft;
        bool                                mMouseMiddle;

        // oiv

        McHandle < SoSerialSectionAligner > mAlignerTool;
        McHandle < SoSeparator >            mRootNode;

        // pointer on data

        HxSerialSectionStack*               mSerialSectionStack;

        // annotations

        McHandle< HxAnnotation >            mAnnotation;

        // tool members

        int                                 mCheckCur;
        McDArray< int >                     mCheckEnds;
        McHandle< HxUniformScalarField3 >   mCheckField;
        int                                 mCheckMax;
        bool                                mCheckNext;
        int                                 mCheckPos;
};

