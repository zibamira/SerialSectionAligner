#pragma once

#include <hxspindle/api.h>
#include <hxspindle/HxSerialSectionStack.h>
#include <mclib/McVec2.h>
#include <mclib/McVec4.h>

#include <hxglutils/HxGLShader.h>
#include <hxspatialgraph/internal/HxSpatialGraph.h>
#include <hxalignmicrotubules/MovingLeastSquares.h>




class HXSPINDLE_API AlignerToolRendererMT
{

    private:


        typedef HxSerialSectionStack::Landmarks        Landmarks;
        typedef HxSerialSectionStack::Matching         Matching;
        typedef HxSerialSectionStack::SectionInterface SectionInterface;
        typedef HxSerialSectionStack::SettingType      SettingType;


    public:


        AlignerToolRendererMT();


    public:


        float computeDistance(const McVec3f& point, McVec2i& microtubuleID);


        void computeIntersectionPoints(McDArray< McVec4f >& points, const int microtubleID, const float radius) const;


        float computeMatchingTransparency(int end1, int end2, float v1, float v2) const;


        McVec4f computePlane(const McVec2i& microtubleID, McVec3f& center);


        const McVec3f& getColor() const;


        const Matching* getMatching() const;


        int getMatch (const McVec4i& match) const;


        float getRadius() const;


        void render3D(unsigned int contextID);


        void render(unsigned int contextID, bool matchingColors);


        void renderDynamic(unsigned int contextID, bool matchingColors);


        void renderMatchings(unsigned int contextID, AlignerToolRendererMT& microtubulesSource);


        void renderStatic3D(unsigned int  contextID);


        void renderWarped(unsigned int contextID, bool matchingColors);


        void renderWarped3D(unsigned int contextID);


        void set(const HxSpatialGraph* microtubules, const Matching* matching, int matchingSet);


        void setBorder(bool border);


        void setClipSphere(const McVec4f& sphere);


        void setColor(const McVec3f& color);


        void setMLS(const MovingLeastSquares& mlsInv);


        void setMLSQuality(int quality);


        void setRadius(float radius);


    private:


        float computeDistance(const McVec3f& point, const McVec3f& lineStart, const McVec3f& lineEnd);


        float computeSliceDistance(const McVec3f& point) const;


        void setSize(int size, int reserve);


        /**
            The general microtubule render function. All previously
            computed points and cylinders are rendered either with the
            constant color or the matching color.
        */
        void render(unsigned int contextID, float radius, bool matchingColors = false);


        /**
            Updates the geometry for all microtubules without warping.
        */
        void update();


        /**
            Updates the geometry for all microtubules with warping every time
            based on z-slices (not arbitrary slices).
        */
        void updateDynamic();


        /**
            Updates the visual geometry for matching visualization.
        */
        void updateMatchings(AlignerToolRendererMT& microtubulesSource);


        /**
            Updates the geometry for all microtubules with warping.
        */
        void updateWarped();


    public:


        McVec3f                         mColor;
        McDArray< McVec3f >             mCylinders;
        McDArray< McVec3f >             mCylinderColors;
        McDArray< int >                 mCylinderMap;
        McDArray< McVec3f >             mCylinderMatchings;
        McDArray< float >               mCylinderPositions;
        const Matching*                 mMatching;
        McDArray< McVec4 < float > >    mMatchingColors;
        int                             mMatchingSet;
        const MovingLeastSquares*       mMLSInv;
        int                             mMLSQuality;
        const HxSpatialGraph*           mMT;
        McDArray< McVec3f >             mPoints;
        McDArray< McVec3f >             mPointColors;
        McDArray< float >               mPointPositions;
        float                           mRadius;
        HxGLShader                      mShaderPoints;
        HxGLShader                      mShaderPoints3D;
        HxGLShader                      mShaderCylinder;
        HxGLShader                      mShaderCylinder3D;
        McVec4f                         mSlice;
        McVec4f                         mSphere;
        bool                            mUpdate;
};


