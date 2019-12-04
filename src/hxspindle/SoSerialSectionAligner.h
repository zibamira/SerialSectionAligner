#ifndef HXSPINDLE_SOSERIALSECTIONALIGNER_H
#define HXSPINDLE_SOSERIALSECTIONALIGNER_H

#include <hxspindle/api.h>
#include <hxspindle/AlignerToolRendererMT.h>
#include <hxspindle/AlignerToolRendererSlice.h>
#include <hxspindle/AlignerToolRendererStack.h>
#include <hxspindle/HxSerialSectionStack.h>

#include <hxfield/HxUniformScalarField3.h>

#include <mclib/McVec3.h>
#include <mclib/McBitfield.h>
#include <mclib/McDArray.h>

#include <hxspatialgraph/internal/HxSpatialGraph.h>

#include <mcgl/internal/mcgl.h>

#include <Inventor/nodes/SoShape.h>
#include <Inventor/actions/SoRayPickAction.h>

#include <hxglutils/HxGLFrameBufferObject.h>
#include <hxglutils/HxGLShader.h>
#include <hxglutils/HxGLVertexBufferObject.h>


class HXSPINDLE_API SoSerialSectionAligner : public SoShape
{
    SO_NODE_HEADER(SoSerialSectionAligner);


    private:


        typedef HxSerialSectionStack::Landmarks      Landmarks;
        typedef HxSerialSectionStack::Matching       Matching;



        enum Interaction
        {
            NONE,
            BLOCKED,
            TRANSLATE,
            ZOOM
        };



        enum RenderMode
        {
            ALIGNMENT,
            MATCHING
        };


    // constructor / destructor

    public:


        SoSerialSectionAligner(void);


        virtual ~SoSerialSectionAligner(void);


    // methods

    public:


        /**
            Computes the intersection area of the two sections.
        */
        float computeIntersectionArea();

        /**
            Returns the section (0 = base section,
            1 = transformed section).
        */
        HxSpatialGraph* getGraph(int sectionID);


        /**
            Returns a picked ending.
        */
        int getPickedEnding(int mouseX, int mouseY, bool control);


        /**
            Returns the section (0 = base section,
            1 = transformed section).
        */
        HxUniformScalarField3* getSection(int sectionID);


        /**
            Returns the selected section change.
        */
        int getSectionChange() const;


        /**
            Returns the slice position in z in the range of [0,...,1].
        */
        float getSlicePosition(int target) const;

        /**
            Returns a slice renderer. Currently two
            renderer are available (0 and 1) for the two
            neighbored sections.
        */
        AlignerToolRendererSlice& getSliceRenderer(int index);


        /**
            Returns the translation of the data.
        */
        McVec2f& getTranslation();


        /**
            Hides the warping grid.
        */
        void hideWarpingGrid();


        /**
            Initialize class
        */
        static void initClass();


        /**
            Controls the landmark setting via mouse. The method
            returns true if a render update is necessary.
        */
        bool interaction(SbVec2s mousePos, bool mouseLeft, bool mouseMidlle, bool controlPress);


        /**
            Sets the check values and the field.
        */
        void setCheck(McHandle< HxUniformScalarField3 > field, const McDArray< int >& ends);


        /**
            Sets the scale factor for the sphere radii which
            represent the landmarks during rendering.
        */
        void setLandmarkScale(float scale);


        /**
            Sets the scale factor for the radii of the
            microtubules rendering.
        */
        void setMicrotubulesScale(float scale);


        /**
            Sets the render mode.

            mode 0 = alignment mode
            mode 1 = matching mode
        */
        void setRenderMode(int mode);


        /**
            Sets the current selected section change.
        */
        void setSectionChange(int change);


        /**
            Sets a selected microtubules end.
        */
        void setSelection(const McVec2i& microtubleEnd);


        /**
            Sets the position of the slice in (0...1) in z-direction.
            To change the bottom slice set target to 0 and for the
            top slice set target to 1.
        */
        void setSlicePosition(float slicePosition, int target);


        /**
            Sets the quality for the slice renderings for the static
            and the dynamic case during interactions. The following
            values are allowed:

                0 : high quality (full resolution)
                1 : medium quality (half resolution)
                2 : low quality (one fourth of the resolution)
        */
        void setSliceQuality(int qualityStatic, int qualityDynamic);


        /**
            Sets the stack data.
        */
        void setStack(HxSerialSectionStack* stack, int change = -1);


        /**
            Sets the numbers of MLS neighbors which influences
            the warping quality.
        */
        void setWarpingQuality(int quality);


        /**
            Shows the warping grid.
        */
        void showWarpingGrid();


        /**
            Updates the end point histogramm for the current interface.
        */
        void updateEndPointDensities();


        /**
            Updates moving least squares for warping.
        */
        void updateMLS(const Landmarks& landmarks);


    protected:


        /**
            Default render function.
        */
        virtual void GLRender(SoGLRenderAction *action);


        // generates primitives (in our case nothing)
        virtual void generatePrimitives(SoAction *action);


        // computes the bounding box
        virtual void computeBBox(SoAction *action, SbBox3f &box, SbVec3f &center);


        // computes picking
        virtual void rayPick(SoRayPickAction *action);


        /**
            Sets the divisor for the slice resolution (see
            AlignerToolRendererSlice).
        */
        void setSliceResolution(int resolution);


    private:


        /**
            Computes the overall axis aligned bounding box of all
            sections.
        */
        void computeBoundingBox();


        /**
            Computes the end point density in a certain region of a section.
            Only the overlapping region of two neighboring sections is considered.
            The z region is set in the range of (0,...,1] and all end points with
            zMin < z end point <= zMax are considered.
        */
        float computeEndPointDensity(int sourceSection, float intersectionArea, float zMin, float zMax);


        /**
            Prints a GL 4x4 matrix.
        */
        void printMatrix(double* m);


        /**
            Render a frame which decomposes the viewer int four
            equal sized screens.
        */
        void renderFrame();


    // attributes

    public:


        SbVec3f*                          mParamCamPosition;


    private:


        McVec3f                           mBoxMin;
        McVec3f                           mBoxMax;

        Interaction                       mControl;
        McVec2i                           mControlPos;
        McVec2f                           mTranslate;
        McVec2f                           mViewSpace;
        float                             mZoom;

        McHandle< HxUniformScalarField3 > mField[2];
        McHandle< HxUniformScalarField3 > mCheckField;
        McDArray< int >                   mCheckEnds;

        McDArray< McDArray < float > >    mEndPointDensities;

        MovingLeastSquares                mMLS;
        MovingLeastSquares                mMLSInv;

        int                               mSliceResolution;
        int                               mSliceResolutionInteraction;

        double                            mModelviewMatrix[16];
        double                            mProjectionMatrix[16];

        RenderMode                        mRenderMode;

        HxSerialSectionStack*             mSerialSectionStack;
        int                               mSectionChange;

        AlignerToolRendererSlice          mCheckRenderer;
        AlignerToolRendererSlice          mSliceRenderer[2];
        AlignerToolRendererSlice          mSideRenderer[2];
        AlignerToolRendererStack          mStackRenderer;
        AlignerToolRendererSlice          mWarpRenderer;

        AlignerToolRendererMT             mMTDynamic;
        AlignerToolRendererMT             mMTStatic[2];
        AlignerToolRendererMT             mMTWarped;

        float                             mSlicePositions[2];
};

#endif

