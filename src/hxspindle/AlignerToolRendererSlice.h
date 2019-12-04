#pragma once


#include <hxspindle/api.h>
#include <hxspindle/AlignerToolRenderer.h>
#include <hxspindle/AlignerToolRendererMT.h>
#include <mclib/McVec2.h>
#include <mclib/McVec4.h>

#include <hxspatialgraph/internal/HxSpatialGraph.h>
#include <hxalignmicrotubules/MovingLeastSquares.h>




/**
    Render a slice of the field data.
*/
class HXSPINDLE_API AlignerToolRendererSlice : public AlignerToolRenderer
{
    private:


        enum InteractionControl
        {
            CONTROL_DELETE,
            CONTROL_MOVE,
            CONTROL_NEW,
            CONTROL_NONE,
            CONTROL_MOUSE_DOWN,
            CONTROL_ROTATE,
            CONTROL_TRANSLATE,
            CONTROL_MINMAX
        };




        enum RenderMode
        {
            ALIGNMENT,
            MATCHING
        };




        typedef HxSerialSectionStack::Landmarks        Landmarks;
        typedef HxSerialSectionStack::Matching         Matching;
        typedef HxSerialSectionStack::SectionInterface SectionInterface;
        typedef HxSerialSectionStack::SettingType      SettingType;




        class Transformation
        {
            public:


                Transformation();


            public:


                void bind() const;


                void compute(const McBox3f& box, const MovingLeastSquares& mls);


                void disable();


                void enable();


                bool isEnabled() const;


                void set(const McVec2d& rotationPoint, const float rotationAngle, const McVec2d& translation);


                McVec2f transform(const McVec2f& point) const;


                McVec2f transformInv(const McVec2f& point) const;


                McVec3f transform(const McVec3f& point) const;


                McVec3f transformInv(const McVec3f& point) const;


                void unbind() const;


            private:


                bool    mEnabled;
                double  mRotationAngle;
                McVec2d mRotationPoint;
                McVec2d mTranslation;
        };


    public:


        AlignerToolRendererSlice(void);


    public:


        /**
            Computes a 2D rigid transformation based on the center and
            a vector of the input box and their transformation by MLS.
            The rigid transformation is then applied to all renderings
            and used for correct interaction.
        */
        void computeTransformation(const McBox3f& box);


        /**
            Returns a selected filament.
        */
        const McVec2i& getSelectedMT() const;


        /**
            Returns the slice position in full z in the range of [0, 1].
            Target 0 is the slice for the constant field and 1 for
            the warped.
        */
        float getSlice(int target) const;


        /**
            Returns the transformation.
        */
        Transformation& getTransformation();


        /**
            Hides the grid and the warped grid.
        */
        void hideGrid();


        /**
            Controls the landmark setting via mouse.
            Left click creates a landmark, left click
            on a landmark with dragging moves a landmark,
            and left click on a landmark with pressed
            control button removes a landmark. Section
            describes the current section ID (either 0
            or one).
        */
        bool landmarkInteraction(SbVec2s           mousePos,
                                 bool              mouseLeft,
                                 bool              controlPress,
                                 SectionInterface& sectionChange,
                                 int               section);


        /**
            Controls the matching setting via mouse.
        */
        bool matchingInteraction(SbVec2s                mousePos,
                                 bool                   mouseLeft,
                                 bool                   mouseMiddle,
                                 bool                   controlPress,
                                 SectionInterface&      sectionChange,
                                 AlignerToolRendererMT& filaments1,
                                 AlignerToolRendererMT& filaments2,
                                 McVec2f                slicePositions,
                                 McVec2f&               translate);

        /**
            Controls the rotation for the 3D view.
        */
        bool matchingInteraction3D(SbVec2s mousePos, bool mouseLeft, bool mouseMiddle);



        /**
            Render 3D view.
        */
        void render3D(unsigned int                         contextID,
                      AlignerToolRendererMT&               filaments1,
                      AlignerToolRendererMT&               filaments2,
                      const float*                         sliceValues);


        /**
            Renders a bar chart with the end point densities.
        */
        void renderEndPointDensities(unsigned int contextID);


        /**
            Render landmarks.
        */
        void renderLandmarks(unsigned int contextID, const Landmarks& landmarks, int sectionID);


        /**
            Render all matching as connecting cylinders.
        */
        void renderMatchings(unsigned int contextID, AlignerToolRendererMT& filaments1, AlignerToolRendererMT& filaments2);


        /**
            Render filaments.
        */
        void renderFilaments(unsigned int contextID, AlignerToolRendererMT& filaments);


        /**
            Render filaments warped.
        */
        void renderFilamentsWarped(unsigned int contextID, AlignerToolRendererMT& filaments);


        /**
            Render planes.
        */
        void renderPlanes(unsigned int contextID, AlignerToolRendererMT& filaments);


        /**
            Render side view.
        */
        void renderSide(unsigned int           contextID,
                        AlignerToolRendererMT& filaments1,
                        AlignerToolRendererMT& filaments2,
                        const float*           sliceValues,
                        bool                   ortho = false);


        /**
            Render slice of the field.
        */
        void renderSlice(unsigned int contextID, const McVec4f& color);


        /**
            Render slice of the field with masked checks.
        */
        void renderSlice(unsigned int contextID, HxUniformScalarField3* checkField, McDArray< int >& checkEnds, Matching& matching, const McVec4f& color);


        /**
            Render transformed slice of the field. The method
            expects a call of "renderSlice" before this method.
        */
        void renderSliceWarped(unsigned int contextID);


        /**
            Sets the radius of the cutting sphere for the 3D view.
        */
        void set3DRadius(float radius);


        /**
            Sets end point densities.
        */
        void setEndPointDensities(const McDArray< float >& densities);


        /**
            Sets the two neighboring sections.
        */
        void setFields(HxUniformScalarField3* fieldConstant, HxUniformScalarField3* fieldWaped);


        /**
            Sets the radii for the spheres that
            represent the landmarks during rendering.
        */
        void setLandmarkRadius(float radius);


        /**
            Sets the scale factor for the sphere radii which
            represent the landmarks during rendering.
        */
        void setLandmarkScale(float scale);


        /**
            Sets the warping-
        */
        void setMLS(const MovingLeastSquares& mls);


        /**
            Sets the number of warping points, which influences
            the warping quality. The value is internally clamped
            to the range of [3, 20]
        */
        void setMLSWarpingPoints(int numPoints);


        /**
            Sets the render mode.

            mode 0 = alignment mode
            mode 1 = matching mode
        */
        void setRenderMode(int mode);


        /**
            Sets the selected filament end.
        */
        void setSelection(const McVec2i& filamentEnd);


        /**
            Sets the full z value of a slice in the range of [0, 1].
            Tagret 0 is the slice for the constant field and 1 for
            the warped.
        */
        void setSlice(float sliceZ, int target);


        /**
            Sets a resolution divisor for the slice rendering. One
            means full resolution, two half resolution, four one fourth
            of the resolution and so far. The lower the resolution the
            worse is the slice quality but the better is the performance.
            values between one and four are suggested.
        */
        void setSliceResolution(int resolution);


        /**
            Shows the grid and the warped grid.
        */
        void showGrid();


        /**
            Controls the state map.
        */
        bool stateMapInteraction(SbVec2s                mousePos,
                                 bool                   mouseLeft,
                                 bool                   mouseMiddle,
                                 bool                   controlPress,
                                 SectionInterface&      sectionChange,
                                 AlignerToolRendererMT& filaments,
                                 McVec2f                slicePositions,
                                 float&                 slicePosition,
                                 HxUniformScalarField3* checkField,
                                 McDArray< int >&       checkEnds,
                                 McVec2f&               translate);



    private:


        /**
            Computes the 3-dimensional position of a point on
            the slice for a given relative tool coordinate [-1,1].
            The method returns false if the view coordinate
            does not correspond to the slice.
        */
        bool computeRelativeSlicePosition(const SbVec2f &toolPosition, SbVec3f &slicePosition) const;


        /**
            Computes the 3-dimensional position of a point on
            the slice for a given view coordinate in pixels.
            The method returns false if the view coordinate
            does not correspond to the slice.
        */
        bool computeSlicePosition(const SbVec2s& viewerPosition, SbVec3f& slicePosition) const;


        /**
            Computes the 2-dimensional position of a point on
            an x-y slice for a given view coordinate in pixels.
            The method returns false if the view coordinate
            does not correspond to the slice.
        */
        bool computeSlicePosition(const SbVec2s& viewerPosition, McVec2f& slicePosition) const;


        /**
            Computes the 3-dimensional positions of all points
            in the view space using a rendering mechanism or
            purely CPU. A call of "updateFrameBuffer" is necessary
            before. Additionally, the members mBoxMin, mBoxMax,
            and mSlice should be set correctly.
        */
        void computeSlicePositions(unsigned int contextID, HxUniformScalarField3* field, McVec4f& slice, bool gpu);


        /**
            Computes the 3-dimensional positions of all points
            in the view space. A call of "updateFrameBuffer" is
            necessary before. Additionally, the member mSlice
            should be set correctly. Mode can be either 0 for
            full screen, 1 for the top half and 2 for the bottom
            half.
        */
        void computeSlicePositionsCPU(unsigned int contextID, HxUniformScalarField3* field, McVec4f& slice, int mode);


        /**
            Computes the values in a slice of a field in view space
            and stores the values into a texture for rendering. This
            requires that for each pixel in view space the 3D position
            is computed by "computeSlicePositions". Mode can be either
            0 for the full view space, 1 for the top half and 2 for the
            bottom half.
        */
        void computeSliceValues(HxUniformScalarField3* field, const McVec4f& color, int mode);


        /**
            Computes the values in a slice of a field in view space
            and stores the values into a texture for rendering. This
            requires that for each pixel in view space the 3D position
            is computed by "computeSlicePositions".
        */
        void computeSliceValues(HxUniformScalarField3* field, const McVec4f& color);


        /**
            Computes the values of a slice for rendering in
            view space. Expects that the slice positions are
            pre-computed in "mSlicePositions" by
            "computeSlicePositions".
        */
        void computeSliceValues(HxUniformScalarField3* field, HxUniformScalarField3* checkField, McDArray< int >& checkEnds, Matching& matching, const McVec4f& color);


        /**
            Computes the intersection points of a plane with
            a bounding box in x-y.
        */
        void computeSliceVisXY(const float* box, const McVec4< float >& plane, McVec2f& start, McVec2f& end);


        /**
            Computes a 1D grid that is warped later. The grid
            is selected horizontally in view space based on a grid spacing
            given in pixel. Due to the orthographic projection (which is only
            supported for the renderer) the projected grid is also uniform
            in the space of the slice. The method expects that the slice
            positions are precomputed (in mSlicePositions) for the view
            space in a previous step. The grid is computed in a way that
            each position fits exactly a slice position created by
            "computeSlicePositions()". To keep the grid uniform at the
            boundary and to cover the whole domain, an extrapolation is
            necessary outside of the view space. But due to the uniform
            structure this extrapolation is correct. The method returns
            false if the grid spacing is too large to create a correct
            grid.
        */
        bool computeWarpGrid1D(McDArray< McVec2d >& grid, const int gridSpacing);


        /**
            Computes the squared grid that is warped later. The grid
            is selected in view space based on a grid spacing given in
            pixel. Due to the orthographic projection (which is only
            supported for the renderer) the projected grid is also uniform
            in the space of the slice. The method expects that the slice
            positions are precomputed (in mSlicePositions) for the view
            space in a previous step. The grid is computed in a way that
            each position fits exactly a slice position created by
            "computeSlicePositions()". To keep the grid uniform at the
            boundary and to cover the whole domain, an extrapolation is
            necessary outside of the view space. But due to the uniform
            structure this extrapolation is correct. The method returns
            false if the grid spacing is too large to create a correct
            grid.
        */
        bool computeWarpGrid(McDArray< McVec2d >& grid, SbVec2s& gridSize, const int gridSpacing);


        /**
            Cut positions.
        */
        void cutSlicePositions(const McVec4f& cutSphere);


        /**
            Returns the landmark ID the sphere of which encloses
            the given position. The method returns -1 in case no
            landmark encloses the position.
        */
        int getIntersectingLandmark(const McVec2f& position, const McDArray< McVec2f >& landmarks) const;


        /**
            Returns the z value for a slice position in range [0,1].
        */
        float getZ(int target) const;


        /**
            Interpolates 2D grid positions.
        */
        void interpolateGrid(McDArray< McVec2d >& grid, const SbVec2s& gridSize, int gridSpacing);


        /**
            Interpolates 2D grid positions and allows the user to set a
            new z value.
        */
        void interpolateGrid(McDArray< McVec2d >& grid, const SbVec2s& gridSize, int gridSpacing, bool setZ, float zValue);


        /**
            Returns the intersection of a line segment with a plane.
        */
        bool planeLineSegIsec(const McVec4< float >& plane, const McVec3f& start, const McVec3f& end, McVec3f& isec);


        /**
            Render the grid and the warped grid.
        */
        void renderGrid(unsigned int contextID, const SbVec2s& gridSize, const McDArray< McVec2d >& grid, const McDArray< McVec2d >& gridWarped, const double z);


        /**
            Prints a GL 4x4 matrix.
        */
        void printMatrix(const double* m) const;


        /**
            Prints a GL 4x4 matrix.
        */
        void printMatrix(const McMat4< double >& m) const;


        /**
            Render filaments.
        */
        void renderFilaments(unsigned int contextID, AlignerToolRendererMT& filaments, int mode, bool border, bool matchingColors, bool setSlice = true);


        /**
            rendering of an OpenGL quad in x-y-space
        */
        void renderQuad(float x, float y, float w, float h, float z = 0.0);


        /**
            Render slice warped side view.
        */
        void renderSliceWarpedSide(unsigned int contextID, HxUniformScalarField3* field2);


        /**
            Warps a set of 2D points using moving least squares.
        */
        void warp(const McDArray< McVec2d >& points, McDArray< McVec2d >& warpedPoints);


    private:


        float                        m3DRadius;

        SbVec3f                      mBoxMin;
        SbVec3f                      mBoxMax;

        McDArray< float >            mEndPointDensities;
        bool                         mEndPointDensitiesVisible;

        HxUniformScalarField3*       mFields[2];

        InteractionControl           mInteractionControl;

        McVec3f                      mLandmarkColors[3];
        McVec2f                      mLandmarkMoveDiff;
        float                        mLandmarkRadius;
        float                        mLandmarkScale;
        static int                   mLandmarkSelected;

        McVec4i                      mMatchingSelected;

        McVec2i                      mMousePos;

        const MovingLeastSquares*    mMLS;
        int                          mMLSNeighbors;

        RenderMode                   mRenderMode;

        McVec2f                      mRotate;
        double                       mRotationMat[16];

        McVec2i                      mSelectedMTBottom;

        HxGLShader                   mShaderDeferred3D;
        HxGLShader                   mShaderAntialiasing;
        HxGLShader                   mShaderLandmarks;
        HxGLShader                   mShaderSlicePositions;

        bool                         mShowWarpingGrid;

        float                        mSliceZ[2];
        McDArray < float >           mSliceDepths;
        McDArray < float >           mSliceDepthsTex;
        McDArray < McVec4 < float> > mSlicePositions;
        int                          mSliceResolution;
        HxGLTexture2D                mSliceTexture;
        HxGLTexture2D                mSliceTextureDepths;
        McDArray < unsigned char >   mSliceValues;

        Transformation               mTransformation;
};


