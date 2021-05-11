#include <hxcore/HxMessage.h>
#include <hxcore/HxResource.h>
#include <hxcore/HxController.h>
#include <hxcore/HxViewer.h>

#include <hxfield/HxLattice3.h>
#include <hxfield/HxLocation3.h>
#include <hxfield/HxUniformCoord3.h>
#include <hxfield/HxUniformScalarField3.h>

#include <hxspindle/SpindleTools.h>
#include <hxspindle/SoSerialSectionAligner.h>

#include <mcgl/internal/mcgl.h>

#include <Inventor/SbBox.h>
#include <Inventor/SbColor.h>
#include <Inventor/SbLinear.h>
#include <Inventor/actions/SoGLRenderAction.h>
#include <Inventor/elements/SoCacheElement.h>

#include <mclib/internal/McWatch.h>
#include <algorithm>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif




SO_NODE_SOURCE(SoSerialSectionAligner);




SoSerialSectionAligner::SoSerialSectionAligner(void)
    : mCheckEnds(0)
    , mControl(NONE)
    , mSectionChange(-1)
    , mSerialSectionStack(0)
    , mSliceResolution(2)
    , mSliceResolutionInteraction(2)
    , mTranslate(0.0f, 0.0f)
    , mZoom(0.0f)
{
    SO_NODE_CONSTRUCTOR(SoSerialSectionAligner);

    mSliceRenderer[0].setSize(1.0, 1.0);
    mSliceRenderer[1].setSize(1.0, 1.0);
    mSliceRenderer[0].setPosition(-1.0, -1.0);
    mSliceRenderer[1].setPosition(-1.0,  0.0);

    mSlicePositions[0] = 0.8;
    mSlicePositions[1] = 0.2;

    mSideRenderer[0].setSize(1.0, 1.0);
    mSideRenderer[0].setPosition(-1.0, -1.0);
    mSideRenderer[1].setSize(1.0, 1.0);
    mSideRenderer[1].setPosition(-1.0,  0.0);

    mStackRenderer.setSize(1.0, 1.0);
    mStackRenderer.setPosition(0.0, -1.0);

    mCheckRenderer.setSize(1.0, 1.0);
    mCheckRenderer.setPosition(0.0, -1.0);

    mWarpRenderer.setSize(1.0, 1.0);
    mWarpRenderer.setPosition(0.0, 0.0);

    mCheckRenderer.setMLS(mMLS);
    mSliceRenderer[0].setMLS(mMLS);
    mSliceRenderer[1].setMLS(mMLS);
    mSideRenderer[0].setMLS(mMLS);
    mSideRenderer[1].setMLS(mMLS);
    mWarpRenderer.setMLS(mMLS);

    mMTDynamic.setMLS(mMLSInv);
    mMTStatic[0].setMLS(mMLSInv);
    mMTStatic[1].setMLS(mMLSInv);
    mMTWarped.setMLS(mMLSInv);

    mMTStatic[0].setColor(1.5 * McVec3f(1.0, 165.0 / 255.0, 0.0));
    mMTStatic[1].setColor(1.5 * McVec3f(0.0, 165.0 / 255.0, 1.0));
    mMTDynamic.setColor(1.5 * McVec3f(0.0, 165.0 / 255.0, 1.0));
    mMTWarped.setColor(1.5 * McVec3f(0.0, 165.0 / 255.0, 1.0));
}




SoSerialSectionAligner::~SoSerialSectionAligner(void)
{
}




void SoSerialSectionAligner::computeBBox(SoAction *action, SbBox3f &box, SbVec3f &center)
{
    box.setBounds(mBoxMin[0], mBoxMin[1], mBoxMin[2], mBoxMax[0], mBoxMax[1], mBoxMax[2]);
    center = box.getCenter();
}




void SoSerialSectionAligner::computeBoundingBox()
{
    if (!mSerialSectionStack) return;

    int n = mSerialSectionStack->getNumSections();

    if (n == 0)
    {
        mBoxMin.setValue(-1.0, -1.0, -1.0);
        mBoxMax.setValue( 1.0,  1.0,  1.0);

        for (int i = 0; i < 2; ++i)
        {
            mSliceRenderer[i].setLandmarkRadius(1.0);
        }
        return;
    }
    else
    {
        mBoxMin.setValue( FLT_MAX,  FLT_MAX,  FLT_MAX);
        mBoxMax.setValue(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }

    for (int i = 0; i < n; ++i)
    {
        const McBox3f box = mSerialSectionStack->getSection(i).mImageBoundingBox;

        mBoxMin[0] = std::min(mBoxMin[0], box[0]);
        mBoxMin[1] = std::min(mBoxMin[1], box[2]);
        mBoxMin[2] = std::min(mBoxMin[2], box[4]);
        mBoxMax[0] = std::max(mBoxMax[0], box[1]);
        mBoxMax[1] = std::max(mBoxMax[1], box[3]);
        mBoxMax[2] = std::max(mBoxMax[2], box[5]);
    }

    float landmarkRadius = (mBoxMax - mBoxMin).length() * 0.005;

    for (int i = 0; i < 2; ++i)
    {
        mSliceRenderer[i].setLandmarkRadius(landmarkRadius);
    }
}




float SoSerialSectionAligner::computeEndPointDensity(int sourceSection, float intersectionArea, float zMin, float zMax)
{
    MovingLeastSquares& mls = sourceSection == 0 ? mMLS : mMLSInv;

    const HxSpatialGraph*        graph1 = 0;
    const HxUniformScalarField3* field2 = 0;

    if (sourceSection == 0)
    {
        graph1 = mSerialSectionStack->getSection(mSectionChange).mFilaments;
        field2 = mField[1];
    }
    else
    {
        graph1 = mSerialSectionStack->getSection(mSectionChange + 1).mFilaments;
        field2 = mField[0];
    }

    if (!graph1 || !field2) return 0.0;

    Matching& matching = mSerialSectionStack->getSectionInterface(mSectionChange).mMatching;

    McBox3f box = field2->getBoundingBox();

    float zGloabalMin = graph1->getBoundingBox().getMin().z;
    float zGloabalMax = graph1->getBoundingBox().getMax().z;
    float zHeight     = zGloabalMax - zGloabalMin;

    int numEnds = 0;

    for (int i = 0; i < graph1->getNumEdges(); ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            if (matching.getState(i, j, sourceSection) == HxSerialSectionStack::UNDEFINED ||
                matching.getState(i, j, sourceSection) == HxSerialSectionStack::UNMATCHED)
            {
                int     v = (j == 0) ? graph1->getEdgeSource(i) : graph1->getEdgeTarget(i);
                McVec3f p = graph1->getVertexCoords(v);

                // point in z range ?

                if (p.z <= zGloabalMin + zMin * zHeight ||
                    p.z >  zGloabalMin + zMax * zHeight)
                {
                    continue;
                }

                // transform to other section and check if end lies in domain

                McVec2d q(p.x, p.y);
                McVec2d qInv = mls.interpolate(q);

                if (qInv.x < box.getMin().x || qInv.x > box.getMax().x ||
                    qInv.y < box.getMin().y || qInv.y > box.getMax().y)
                {
                    continue;
                }

                numEnds++;
            }
        }
    }

    return float(numEnds) / ((zMax- zMin) * zHeight * intersectionArea * 0.001 * 0.000000001);
}




float SoSerialSectionAligner::computeIntersectionArea()
{
    return SpindleTools::computeIntersectionArea(*mField[0], *mField[1], mMLS, 10);
}




HxSpatialGraph* SoSerialSectionAligner::getGraph(int sectionID)
{
    if (!mSerialSectionStack) return 0;

    int n = mSerialSectionStack->getNumSections();

    if (n > mSectionChange + 1 && mSectionChange >= 0)
    {
        if (sectionID == 0)
        {
            return mSerialSectionStack->getSection(mSectionChange).mFilaments;
        }
        else 
        {
            return mSerialSectionStack->getSection(mSectionChange + 1).mFilaments;
        }
    }

    return 0;
}




HxUniformScalarField3* SoSerialSectionAligner::getSection(int sectionID)
{
    if (!mSerialSectionStack) return 0;

    if (sectionID < 0 || sectionID > 1) return 0;

    return mField[sectionID];
}




int SoSerialSectionAligner::getSectionChange() const
{
    return mSectionChange;
}




float SoSerialSectionAligner::getSlicePosition(int target) const
{
    return mSlicePositions[target];
}




AlignerToolRendererSlice& SoSerialSectionAligner::getSliceRenderer(int index)
{
    if (index == 2) return mWarpRenderer;
    else if (index == 3) return mSideRenderer[0];
    else if (index == 4) return mSideRenderer[1];
    else if (index == 5) return mCheckRenderer;

    return mSliceRenderer[index];
}





McVec2f& SoSerialSectionAligner::getTranslation()
{
    return mTranslate;
}




void SoSerialSectionAligner::generatePrimitives(SoAction *action)
{
}




void SoSerialSectionAligner::GLRender(SoGLRenderAction *action)
{
    if (!mField[0] || ! mField[1]) return;

    unsigned int contextID = action->getCacheContext();

    if (!mSerialSectionStack) return;

    if (mSectionChange >= mSerialSectionStack->getNumSections() - 1) return;

    // setup view

    HxViewer* viewer = theController->getCurrentViewer();

    if (!viewer) return;

    SbVec2s size = viewer->getSize();

    const McBox3f& box1 = mField[0]->getBoundingBox();

    float   ratio  = (float) size[0] / (float) size[1];
    float   depth  = (box1.getMax().z - box1.getMin().z);
    float   height = box1.getMax().y - box1.getMin().y;
    McVec3f center = box1.getCenter();

    mViewSpace.setValue(ratio * height, height);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-ratio * (height - mZoom), ratio * (height - mZoom), -(height - mZoom), height - mZoom, -depth, depth);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(-center.x, -center.y, -center.z);
    glTranslatef(mTranslate.x, mTranslate.y, 0.0f);


    // update warping

    const Landmarks& landmarks = mSerialSectionStack->getSectionInterface(mSectionChange).mLandmarks;
          Matching&  matching  = mSerialSectionStack->getSectionInterface(mSectionChange).mMatching;

    updateMLS(landmarks);

    // compute section planes

    McBox3f box = mField[0]->getBoundingBox();

    mSliceRenderer[1].computeTransformation(box);
    mSliceRenderer[0].getTransformation() = mSliceRenderer[1].getTransformation();
    mSliceRenderer[0].getTransformation().disable();

    int n = mSerialSectionStack->getNumSections();

    if (n > mSectionChange + 1 && mSectionChange >= 0)
    {
        if (mRenderMode == ALIGNMENT)
        {
            mSliceRenderer[0].renderSlice(contextID, McVec4f(1.0, 1.0, 1.0, 1.0));
            mSliceRenderer[1].renderSlice(contextID, McVec4f(1.0, 1.0, 1.0, 1.0));

            mSliceRenderer[0].renderFilaments(contextID, mMTStatic[0]);
            mSliceRenderer[1].renderFilaments(contextID, mMTStatic[1]);

            mSliceRenderer[0].renderLandmarks(contextID, landmarks, 0);
            mSliceRenderer[1].renderLandmarks(contextID, landmarks, 1);
        }

        mWarpRenderer.renderSlice(contextID, McVec4f(1.0, 165.0 / 255.0, 0.0, 1.0));
        mWarpRenderer.renderSliceWarped(contextID);

        mWarpRenderer.renderFilaments(contextID, mMTStatic[0]);
        mWarpRenderer.renderFilamentsWarped(contextID, mMTDynamic);

        mWarpRenderer.renderMatchings(contextID, mMTStatic[0], mMTDynamic);

        if (mRenderMode == MATCHING)
        {
            mWarpRenderer.renderPlanes(contextID, mMTStatic[0]);

            mSideRenderer[0].renderSide(contextID, mMTStatic[0], mMTWarped, mSlicePositions);
            //mSideRenderer[1].renderSide(contextID, mMTStatic[0], mMTWarped, mSlicePositions, true);
            mSideRenderer[1].render3D(contextID, mMTStatic[0], mMTWarped, mSlicePositions);
        }
    }

    if (mRenderMode == ALIGNMENT)
    {
        mStackRenderer.renderStack(contextID, mSerialSectionStack, mEndPointDensities);
    }
    else
    {
        mCheckRenderer.renderSlice(contextID, mCheckField, mCheckEnds, matching, McVec4f(1.0, 0.92, 0.7, 1.0));
        mCheckRenderer.renderEndPointDensities(contextID);
    }

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    renderFrame();

    SoCacheElement::invalidate(action->getState());
}




void SoSerialSectionAligner::hideWarpingGrid()
{
    mWarpRenderer.hideGrid();
    touch();
}




void SoSerialSectionAligner::initClass()
{
    SO_NODE_INIT_CLASS(SoSerialSectionAligner, SoShape, "Shape");
}




bool SoSerialSectionAligner::interaction(SbVec2s mousePos, bool mouseLeft, bool mouseMiddle, bool controlPress)
{
    bool interact = false;

    // detect window size information

    HxViewer* viewer = theController->getCurrentViewer();

    if (!viewer) return false;

    SbVec2s windowSize = viewer->getSize();

    // update warping

    if (mSectionChange >= mSerialSectionStack->getNumSections() - 1) return false;

    updateMLS(mSerialSectionStack->getSectionInterface(mSectionChange).mLandmarks);

    // start interaction

    if (mControl == NONE || mControl == BLOCKED)
    {
        for (int i = 0; i < 2; ++i)
        {
            if (mRenderMode == ALIGNMENT)
            {
                if (mSliceRenderer[i].landmarkInteraction(mousePos, mouseLeft, controlPress, mSerialSectionStack->getSectionInterface(mSectionChange), i))
                {
                    touch();
                    interact = true;
                }
            }
        }

        bool interact3D = false;

        if (mRenderMode == MATCHING)
        {
            // compute planes

            McBox3f box1;
            McBox3f box2;

            if (mField[0]) box1 = mField[0]->getBoundingBox();
            if (mField[1]) box2 = mField[1]->getBoundingBox();

            float z1 = mSlicePositions[0];
            float z2 = mSlicePositions[1];

            z1 = (1.0 - z1) * box1[4] + z1 * box1[5];
            z2 = (1.0 - z2) * box2[4] + z2 * box2[5];

            if (mWarpRenderer.matchingInteraction(mousePos, mouseLeft, mouseMiddle, controlPress, mSerialSectionStack->getSectionInterface(mSectionChange), mMTStatic[0], mMTDynamic, McVec2f(z1, z2), mTranslate))
            {
                McVec2i selMT = mWarpRenderer.getSelectedMT();

                setSelection(selMT);

                touch();
                interact = true;
            }

            if (mSideRenderer[1].matchingInteraction3D(mousePos, mouseLeft, mouseMiddle))
            {
                setSliceResolution(mSliceResolutionInteraction);
                touch();
                interact3D = true;
            }
            else
            {
                setSliceResolution(mSliceResolution);
            }

            float slicePos = 0.0f;

            if (mCheckRenderer.stateMapInteraction(mousePos, mouseLeft, mouseMiddle, controlPress, mSerialSectionStack->getSectionInterface(mSectionChange), mMTStatic[0], McVec2f(z1, z2), slicePos, mCheckField, mCheckEnds, mTranslate))
            {
                McVec2i selMT = mCheckRenderer.getSelectedMT();

                mWarpRenderer.setSelection(selMT);

                setSelection(selMT);
                setSlicePosition(slicePos, 0);
                touch();

                interact = true;
            }
        }

        if (mRenderMode == ALIGNMENT)
        {
            if (mStackRenderer.interaction(mousePos, mouseLeft, controlPress, mSerialSectionStack))
            {
                if (mStackRenderer.getSectionChange() != mSectionChange)
                {
                    setSectionChange(mStackRenderer.getSectionChange());
                    touch();
                    interact = true;
                }
            }
        }

        if (interact || interact3D) mControl = BLOCKED;
        else                        mControl = NONE;
    }


    // start translation

    if (mouseLeft && !mouseMiddle && !controlPress && mControl == NONE)
    {
        mControl = TRANSLATE;

        mControlPos.setValue(mousePos[0], mousePos[1]);
    }

    // start zooming

    if (mouseLeft && mouseMiddle && !controlPress && mControl == NONE)
    {
        mControl = ZOOM;

        mControlPos.setValue(mousePos[0], mousePos[1]);
    }

    // stop translation

    if ((!mouseLeft || mouseMiddle) && mControl == TRANSLATE)
    {
        setSliceResolution(mSliceResolution);

        touch();

        mControl = NONE;
    }

    // stop zooming

    if ((!mouseLeft || !mouseMiddle) && mControl == ZOOM)
    {
        setSliceResolution(mSliceResolution);

        touch();

        mControl = NONE;
    }

    // translation

    if (mControl == TRANSLATE)
    {
        float ratio = (float) windowSize[0] / (float) windowSize[1];

        float x = 4.0f * float(mControlPos.x - mousePos[0]) / float(windowSize[0]);
        float y = 4.0f * float(mControlPos.y - mousePos[1]) / float(windowSize[1]);

        mTranslate.x -= ratio * (mViewSpace.y - mZoom) * x;
        mTranslate.y -= (mViewSpace.y - mZoom) * y;

        mControlPos.x = mousePos[0];
        mControlPos.y = mousePos[1];

        setSliceResolution(mSliceResolutionInteraction);

        touch();
    }

    // zooming

    if (mControl == ZOOM)
    {
        float y = 8.0f * float(mControlPos.y - mousePos[1]) / float(windowSize[1]);

        mZoom += ((mViewSpace.y - mZoom)) * y;

        mControlPos.x = mousePos[0];
        mControlPos.y = mousePos[1];

        setSliceResolution(mSliceResolutionInteraction);

        touch();
    }


    return interact;
}




void SoSerialSectionAligner::printMatrix(double* m)
{
    theMsg->printf("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
}




void SoSerialSectionAligner::rayPick(SoRayPickAction *action)
{
}




void SoSerialSectionAligner::renderFrame()
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_LIGHTING);

    glLineWidth(3.0f);
    glColor4f(0.411, 0.517, 0.643, 1.0);

    glBegin(GL_LINES);
        glVertex2f(-1.0f,  0.0f);
        glVertex2f( 1.0f,  0.0f);
        glVertex2f( 0.0f, -1.0f);
        glVertex2f( 0.0f,  1.0f);
    glEnd();

    glLineWidth(1.0f);

    glEnable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}




void SoSerialSectionAligner::setCheck(McHandle< HxUniformScalarField3 > field, const McDArray< int >& ends)
{
    mCheckField = field;
    mCheckEnds  = ends;
}




void SoSerialSectionAligner::setLandmarkScale(float scale)
{
    for (int i = 0; i < 2; ++i)
    {
        mSliceRenderer[i].setLandmarkScale(scale);
    }

    touch();
}




void SoSerialSectionAligner::setMicrotubulesScale(float scale)
{
    mMTStatic[0].setRadius(scale * 90.0);
    mMTStatic[1].setRadius(scale * 90.0);
    mMTDynamic.setRadius(scale * 90.0);
    mMTWarped.setRadius(scale * 90.0);
}





void SoSerialSectionAligner::setRenderMode(int mode)
{
    mSliceRenderer[0].setRenderMode(mode);
    mSliceRenderer[1].setRenderMode(mode);

    mWarpRenderer.setRenderMode(mode);

    if (mode == 0) mRenderMode = ALIGNMENT;
    else           mRenderMode = MATCHING;

    touch();
}





void SoSerialSectionAligner::setSectionChange(int change)
{
    int oldChange = mSectionChange;

    mSectionChange = std::max(0, change);

    if (mSerialSectionStack)
    {
        int n = mSerialSectionStack->getNumSections();

        if (oldChange != mSectionChange)
        {
            if (oldChange >= 0 && oldChange + 1 == mSectionChange)
            {
                mField[0] = mField[1];
                mField[1] = 0;
            }
            else if (oldChange >= 1 && oldChange - 1 == mSectionChange)
            {
                mField[1] = mField[0];
                mField[0] = 0;
            }
            else
            {
                mField[0] = 0;
                mField[1] = 0;
            }

            if (!mField[0] && n > mSectionChange && mSectionChange >= 0)
            {
                mField[0] = mSerialSectionStack->getSection(mSectionChange).getImage();
            }
            if (!mField[1] && n > mSectionChange + 1 && mSectionChange >= 0)
            {
                mField[1] = mSerialSectionStack->getSection(mSectionChange + 1).getImage();
            }
        }

        mCheckRenderer.setFields(mField[0], 0);
        mSliceRenderer[0].setFields(mField[0], 0);
        mSliceRenderer[1].setFields(mField[1], 0);
        mSideRenderer[0].setFields(mField[0], mField[1]);
        mSideRenderer[1].setFields(mField[0], mField[1]);
        mWarpRenderer.setFields(mField[0], mField[1]);


        if (n > mSectionChange + 1 && mSectionChange >= 0)
        {
            const HxSpatialGraph* graph1 = mSerialSectionStack->getSection(mSectionChange).mFilaments;
            const HxSpatialGraph* graph2 = mSerialSectionStack->getSection(mSectionChange + 1).mFilaments;

            Matching& matching = mSerialSectionStack->getSectionInterface(mSectionChange).mMatching;

            matching.updateEndStates(graph1, graph2);

            mMTStatic[0].set(graph1, &matching, 0);
            mMTStatic[1].set(graph2, &matching, 1);
            mMTDynamic.set(graph2, &matching, 1);
            mMTWarped.set(graph2, &matching, 1);
        }

        if (mEndPointDensities[mSectionChange].size() == 0)
        {
            updateEndPointDensities();
        }
        else
        {
            mCheckRenderer.setEndPointDensities(mEndPointDensities[mSectionChange]);
        }

        mStackRenderer.setSectionChange(mSectionChange);
    }

    touch();
}




void SoSerialSectionAligner::setSelection(const McVec2i& microtubleEnd)
{
    mSideRenderer[0].setSelection(microtubleEnd);
    mSideRenderer[1].setSelection(microtubleEnd);
    mWarpRenderer.setSelection(microtubleEnd);
}




void SoSerialSectionAligner::setSlicePosition(float slicePosition, int target)
{
    mSlicePositions[target] = slicePosition;

    if (target == 0)
    {
        mSliceRenderer[0].setSlice(slicePosition, 0);
    }
    else
    {
        mSliceRenderer[1].setSlice(slicePosition, 0);
    }

    mCheckRenderer.setSlice(slicePosition, target);
    mSideRenderer[0].setSlice(slicePosition, target);
    mSideRenderer[0].setSlice(slicePosition, target);
    mWarpRenderer.setSlice(slicePosition, target);

    touch();
}




void SoSerialSectionAligner::setSliceQuality(int qualityStatic, int qualityDynamic)
{
    if      (qualityStatic <= 0) mSliceResolution = 1;
    else if (qualityStatic == 1) mSliceResolution = 2;
    else                         mSliceResolution = 4;

    if      (qualityDynamic <= 0) mSliceResolutionInteraction = 1;
    else if (qualityDynamic == 1) mSliceResolutionInteraction = 2;
    else                          mSliceResolutionInteraction = 4;

    setSliceResolution(mSliceResolution);
    touch();
}



void SoSerialSectionAligner::setSliceResolution(int resolution)
{
    mSliceRenderer[0].setSliceResolution(resolution);
    mSliceRenderer[1].setSliceResolution(resolution);
    mCheckRenderer.setSliceResolution(resolution);
    mSideRenderer[0].setSliceResolution(resolution);
    mSideRenderer[1].setSliceResolution(resolution);
    mWarpRenderer.setSliceResolution(resolution);
}




void SoSerialSectionAligner::setStack(HxSerialSectionStack* stack, int change)
{
    mSerialSectionStack = stack;

    if (!mSerialSectionStack)
    {
        mSectionChange = -1;
        mField[0]      = 0;
        mField[1]      = 0;
    }
    else
    {
        computeBoundingBox();

        // initialize end point densities

        mEndPointDensities.resize(mSerialSectionStack->getNumSections() - 1);

        for (int i = 0; i < mEndPointDensities.size(); ++i)
        {
            mEndPointDensities[i].clear();
        }

        // force an update

        if (change < 0)
        {
            int oldChange = mSectionChange;
            mSectionChange = -1;
            setSectionChange(oldChange);
        }
        else
        {
            mSectionChange = -1;
            setSectionChange(change);
        }
    }

    touch();
}




void SoSerialSectionAligner::setWarpingQuality(int quality)
{
    mSliceRenderer[0].setMLSWarpingPoints(quality);
    mSliceRenderer[1].setMLSWarpingPoints(quality);
    mWarpRenderer.setMLSWarpingPoints(quality);
    mSideRenderer[0].setMLSWarpingPoints(quality);
    mSideRenderer[1].setMLSWarpingPoints(quality);
    mCheckRenderer.setMLSWarpingPoints(quality);

    mMTDynamic.setMLSQuality(quality);
    mMTWarped.setMLSQuality(quality);
    mMTStatic[0].setMLSQuality(quality);
    mMTStatic[1].setMLSQuality(quality);

    touch();
}




void SoSerialSectionAligner::showWarpingGrid()
{
    mWarpRenderer.showGrid();
    touch();
}




void SoSerialSectionAligner::touchMT()
{
    mMTDynamic.touch();
    mMTStatic[0].touch();
    mMTStatic[1].touch();
    mMTWarped.touch();
}




void SoSerialSectionAligner::updateEndPointDensities()
{
    float area = SpindleTools::computeIntersectionArea(*mField[0], *mField[1], mMLS, 10);

    float h[4];

    h[0] = computeEndPointDensity(0, area, 0.5, 0.75);
    h[1] = computeEndPointDensity(0, area, 0.75, 1.0);
    h[2] = computeEndPointDensity(1, area, 0.0, 0.25);
    h[3] = computeEndPointDensity(1, area, 0.25, 0.5);

    for (int i = 3; i >= 0; --i)
    {
        mEndPointDensities[mSectionChange].push_back(h[i]);
    }

    mCheckRenderer.setEndPointDensities(mEndPointDensities[mSectionChange]);
}




void SoSerialSectionAligner::updateMLS(const Landmarks& landmarks)
{
    int n = landmarks.getNumMarks();

    McDArray< McVec2d > source(n);
    McDArray< McVec2d > target(n);

    for (int i = 0; i < n; ++i)
    {
        source[i].setValue(landmarks.getMarks(0)[i][0], landmarks.getMarks(0)[i][1]);
        target[i].setValue(landmarks.getMarks(1)[i][0], landmarks.getMarks(1)[i][1]);
    }

    mMLS.setLandmarks(source, target);
    mMLSInv.setLandmarks(target, source);

    mMLS.initializeGrid();
    mMLSInv.initializeGrid();
}
