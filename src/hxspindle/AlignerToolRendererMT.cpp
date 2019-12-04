#include <hxspindle/AlignerToolRendererMT.h>
#include <hxspindle/SpindleTools.h>

#include <hxcore/HxMessage.h>
#include <hxcore/HxResource.h>
#include <hxcore/HxController.h>
#include <hxcore/HxViewer.h>

#include <mclib/McVec3.h>
#include <mclib/internal/McWatch.h>
#include <mclib/McRot.h>

#ifdef _OPENMP
#   include <omp.h>
#endif




static void computeEdgeColors(int numEdges, const HxSerialSectionStack::Matching& matching, int matchingSet, McDArray< McVec3f >* edgeColors)
{
    edgeColors[0].resize(numEdges);
    edgeColors[1].resize(numEdges);

    edgeColors[0].fill(McVec3f(0.1f, 0.1f, 0.1f));
    edgeColors[1].fill(McVec3f(0.1f, 0.1f, 0.1f));

    for (int i = 0; i < matching.getNumMatches(); ++i)
    {
        McVec3f c = matching.getColor(i);
        int     m = matching.get(i).i;
        int     e = matching.get(i).j;

        if (matchingSet == 1)
        {
            m = matching.get(i).k;
            e = matching.get(i).l;
        }

        edgeColors[e][m] = c;
    }
}



AlignerToolRendererMT::AlignerToolRendererMT()
    : mMatching(0)
    , mMLSInv(0)
    , mMT(0)
    , mRadius(90.0)
    , mUpdate(false)
{
    mShaderCylinder.load("/share/shaders/hxspindle/CylinderCutOrtho", 4, GL_POINTS, GL_TRIANGLE_STRIP);
    mShaderPoints.load("/share/shaders/hxspindle/SphereCutOrtho", 4, GL_POINTS, GL_TRIANGLE_STRIP);
    mShaderPoints3D.load("/share/shaders/hxspindle/SphereOrtho", 4, GL_POINTS, GL_TRIANGLE_STRIP);
    mShaderCylinder3D.load("/share/shaders/hxspindle/CylinderOrtho", 4, GL_POINTS, GL_TRIANGLE_STRIP);
}




float AlignerToolRendererMT::computeDistance(const McVec3f& point, McVec2i& microtubuleID)
{
    int     n = mCylinders.size() / 2;
    float   d = FLT_MAX;
    McVec3f p = point;

    microtubuleID.setValue(-1, -1);

    // detect closest visible microtuble

    for (int i = 0; i < n; ++i)
    {
        McVec3f s = mCylinders[i * 2 + 0];
        McVec3f e = mCylinders[i * 2 + 1];

        float d_i = computeDistance(p, s, e);

        if (d_i < d)
        {
            d = d_i;
            microtubuleID.x = mCylinderMap[i];

            if (mCylinderPositions[i] < 0.5)
            {
                microtubuleID.y = 0;
            }
            else
            {
                microtubuleID.y = 1;
            }
        }
    }

    // determine microtuble end closer to the point

    /*if (microtubuleID.x >= 0)
    {
        McVec3f p = mMT->getEdgePoint(microtubuleID.x, 0);
        McVec3f q = mMT->getEdgePoint(microtubuleID.x, mMT->getNumEdgePoints(microtubuleID.x) - 1);

        if ((point - p).length2() < (point - q).length2())
        {
            microtubuleID.y = 0;
        }
        else
        {
            microtubuleID.y = 1;
        }
    }*/

    return d;
}




float AlignerToolRendererMT::computeDistance(const McVec3f& point, const McVec3f& lineStart, const McVec3f& lineEnd)
{
    McVec3f rS = lineStart;
    McVec3f rD = lineEnd - lineStart;
    McVec3f p  = point;

    double t   = rD.dot(p - rS) / rD.dot(rD);

    if (t < 0.0)
    {
        return (lineStart-point).length();
    }
    else if (t > 1.0)
    {
        return (lineEnd - point).length();
    }
    else
    {
        return ((rS + t * rD) - point).length();
    }
}




void AlignerToolRendererMT::computeIntersectionPoints(McDArray< McVec4f >& points, const int microtubleID, const float radius) const
{
    points.clear();

    if (!mMT) return;

    int numPoints = mMT->getNumEdgePoints(microtubleID);

    float edgeLength = 0.0;
    float currLength = 0.0;

    for (int i = 0; i < numPoints - 1; ++i)
    {
        McVec3f p = mMT->getEdgePoint(microtubleID, i);
        McVec3f q = mMT->getEdgePoint(microtubleID, i + 1);

        edgeLength += (p - q).length();
    }

    McVec3f n(mSlice.x, mSlice.y, mSlice.z);
    float   d(mSlice.t);
    float   l = n.length();

    n *= (1.0 / l);
    d *= l;

    for (int i = 0; i < numPoints - 1; ++i)
    {
        McVec3f p = mMT->getEdgePoint(microtubleID, i);
        McVec3f q = mMT->getEdgePoint(microtubleID, i + 1);

        currLength += (p - q).length();

        float t = (d - n.dot(p)) / n.dot(q - p);

        if (t >= 0.0 && t <= 1.0)
        {
            p = p + t * (q - p);
            points.push(McVec4f(p.x, p.y, p.z, currLength / edgeLength));
        }
    }

    if (points.size() == 0)
    {
        McVec3f p = mMT->getEdgePoint(microtubleID, 0);
        McVec3f q = mMT->getEdgePoint(microtubleID, numPoints - 1);

        if (fabs(n.dot(p) - d) < radius)
        {
            McVec3f x = p + ((d - n.dot(p)) / n.dot(n)) * n;
            points.push(McVec4f(x.x, x.y, x.z, 0.0));
        }

        if (fabs(n.dot(q) - d) < radius)
        {
            McVec3f x = q + ((d - n.dot(q)) / n.dot(n)) * n;
            points.push(McVec4f(x.x, x.y, x.z, 1.0));
        }
    }
}




float AlignerToolRendererMT::computeMatchingTransparency(int end1, int end2, float v1, float v2) const
{
    float t1 = std::min(1.0f, std::max(0.0f, v1));
    float t2 = std::min(1.0f, std::max(0.0f, v2));

    if (end1 == 0) t1 = 1.0 - t1;
    if (end2 == 0) t2 = 1.0 - t2;

    float t = std::min(t1, t2);

    t = std::max(0.0, (t - 0.5f) * 2.0);

    return pow(t, 0.3f);
}





McVec4f AlignerToolRendererMT::computePlane(const McVec2i& microtubleID, McVec3f& center)
{
    if (microtubleID.x < 0)
    {
        center.setValue(0.0f, 0.0f, 0.0f);

        return McVec4f(1.0f, 0.0f, 0.0f, 0.0f);
    }

    McVec3f p;
    McVec3f q;

    int n = mMT->getNumEdgePoints(microtubleID.x);

    if (n < 2) return McVec4f(0.0, 0.0, 1.0, 0.0);

    if (microtubleID.y == 0)
    {
        p = mMT->getEdgePoint(microtubleID.x, 0);
        //q = mMT->getEdgePoint(microtubleID.x, 1);
        q = SpindleTools::computePointAtDist(mMT->getEdgePoints(microtubleID.x), 2000.0f, 1, 1);
    }
    else
    {
        p = mMT->getEdgePoint(microtubleID.x, n - 1);
        //q = mMT->getEdgePoint(microtubleID.x, n - 2);
        q = SpindleTools::computePointAtDist(mMT->getEdgePoints(microtubleID.x), 2000.0f, n - 2, -1);
    }

    McVec3f planeNormal = (p - q).cross(McVec3f(0.0, 0.0, 1.0));

    if (planeNormal.length() <= 0.0000001)
    {
        planeNormal.setValue(-1.0f, 0.0f, 0.0f);
    }

    planeNormal.normalize();

    McVec4f plane(planeNormal.x, planeNormal.y, planeNormal.z, planeNormal.dot(p));

    center = p;

    return plane;
}




const AlignerToolRendererMT::Matching* AlignerToolRendererMT::getMatching() const
{
    return mMatching;
}




float AlignerToolRendererMT::computeSliceDistance(const McVec3f& point) const
{
    return mSlice[0] * point.x + mSlice[1] * point.y + mSlice[2] * point.z - mSlice[3];
}




const McVec3f& AlignerToolRendererMT::getColor() const
{
    return mColor;
}




int AlignerToolRendererMT::getMatch(const McVec4i& match) const
{
    for (int i = 0; i < mMatching->getNumMatches(); ++i)
    {
        if (mMatching->get(i) == match) return i;
    }

    return -1;
}




float AlignerToolRendererMT::getRadius() const
{
    return mRadius;
}




void AlignerToolRendererMT::render3D(unsigned int contextID)
{
    bool matchingColors = true;

    glColor3f(mColor.x, mColor.y, mColor.z);

    // points

    if (mPoints.size() > 0)
    {
        mShaderPoints3D.enable(contextID);
        mShaderPoints3D.setParameter1f(contextID, "radius", mRadius);
        mShaderPoints3D.setParameter4f(contextID, "view_sphere", &mSphere[0]);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float) * 3, mPoints.dataPtr());

        if (matchingColors)
        {
            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(3, GL_FLOAT, sizeof(float) * 3, mPointColors.dataPtr());
        }

        glDrawArrays(GL_POINTS, 0, mPoints.size());

        if (matchingColors)
        {
            glDisableClientState(GL_COLOR_ARRAY);
        }

        glDisableClientState(GL_VERTEX_ARRAY);

        mShaderPoints.disable();
    }

    // cylinder

    if (mCylinders.size() > 0)
    {
        mShaderCylinder3D.enable(contextID);
        mShaderCylinder3D.setParameter1f(contextID, "radius", mRadius);
        mShaderCylinder3D.setParameter4f(contextID, "view_sphere", &mSphere[0]);

        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glClientActiveTextureARB(GL_TEXTURE0_ARB);
        glTexCoordPointer(3, GL_FLOAT, sizeof(float) * 6, &mCylinders[1]);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float) * 6, mCylinders.dataPtr());

        if (matchingColors)
        {
            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(3, GL_FLOAT, sizeof(float) * 3, mCylinderColors.dataPtr());
        }

        glDrawArrays(GL_POINTS, 0, mCylinders.size() / 2);

        if (matchingColors)
        {
            glDisableClientState(GL_COLOR_ARRAY);
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);

        mShaderCylinder3D.disable();
    }
}




void AlignerToolRendererMT::render(unsigned int contextID, bool matchingColors)
{
    update();

    glDisable(GL_DEPTH_TEST);

    render(contextID, mRadius, matchingColors);

    glEnable(GL_DEPTH_TEST);
}




void AlignerToolRendererMT::render(unsigned int contextID, float radius, bool matchingColors)
{
    glColor3f(mColor.x, mColor.y, mColor.z);

    // points

    if (mPoints.size() > 0)
    {
        mShaderPoints.enable(contextID);
        mShaderPoints.setParameter1f(contextID, "radius", radius);
        mShaderPoints.setParameter4f(contextID, "slice", &mSlice[0]);
        mShaderPoints.setParameter1f(contextID, "sliceHeight", 1.0);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float) * 3, mPoints.dataPtr());

        if (matchingColors)
        {
            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(3, GL_FLOAT, sizeof(float) * 3, mPointColors.dataPtr());
        }

        glDrawArrays(GL_POINTS, 0, mPoints.size());

        if (matchingColors)
        {
            glDisableClientState(GL_COLOR_ARRAY);
        }

        glDisableClientState(GL_VERTEX_ARRAY);

        mShaderPoints.disable();
    }

    // cylinder

    if (mCylinders.size() > 0)
    {
        mShaderCylinder.enable(contextID);
        mShaderCylinder.setParameter1f(contextID, "radius", radius);
        mShaderCylinder.setParameter4f(contextID, "slice", &mSlice[0]);
        mShaderCylinder.setParameter1f(contextID, "sliceHeight", 1.0);

        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glClientActiveTextureARB(GL_TEXTURE0_ARB);
        glTexCoordPointer(3, GL_FLOAT, sizeof(float) * 6, &mCylinders[1]);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float) * 6, mCylinders.dataPtr());

        if (matchingColors)
        {
            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(3, GL_FLOAT, sizeof(float) * 3, mCylinderColors.dataPtr());
        }

        glDrawArrays(GL_POINTS, 0, mCylinders.size() / 2);

        if (matchingColors)
        {
            glDisableClientState(GL_COLOR_ARRAY);
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);

        mShaderCylinder.disable();
    }
}




void AlignerToolRendererMT::renderDynamic(unsigned int contextID, bool matchingColors)
{
    updateDynamic();

    glDisable(GL_DEPTH_TEST);

    render(contextID, mRadius, matchingColors);

    glEnable(GL_DEPTH_TEST);
}




void AlignerToolRendererMT::renderMatchings(unsigned int contextID, AlignerToolRendererMT& microtubulesSource)
{
    updateMatchings(microtubulesSource);

    glColor3f(1.0f, 0.0f, 0.0f);

    // points

    if (mCylinderMatchings.size() > 0)
    {
        mShaderPoints.enable(contextID);
        mShaderPoints.setParameter1f(contextID, "radius", mRadius);
        mShaderPoints.setParameter4f(contextID, "slice", &mSlice[0]);
        mShaderPoints.setParameter1f(contextID, "sliceHeight", 1.0);

        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(4, GL_FLOAT, sizeof(float) * 4, mMatchingColors.dataPtr());

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float) * 3, mCylinderMatchings.dataPtr());

        glDrawArrays(GL_POINTS, 0, mCylinderMatchings.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);

        mShaderPoints.disable();
    }

    // cylinder

    if (mCylinderMatchings.size() > 0)
    {
        mShaderCylinder.enable(contextID);
        mShaderCylinder.setParameter1f(contextID, "radius", mRadius * 0.3);
        mShaderCylinder.setParameter4f(contextID, "slice", &mSlice[0]);
        mShaderCylinder.setParameter1f(contextID, "sliceHeight", 1.0);

        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer(4, GL_FLOAT, sizeof(float) * 8, mMatchingColors.dataPtr());

        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glClientActiveTextureARB(GL_TEXTURE0_ARB);
        glTexCoordPointer(3, GL_FLOAT, sizeof(float) * 6, &mCylinderMatchings[1]);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(float) * 6, mCylinderMatchings.dataPtr());

        glDrawArrays(GL_POINTS, 0, mCylinderMatchings.size() / 2);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);

        mShaderCylinder.disable();
    }
}




void AlignerToolRendererMT::renderStatic3D(unsigned int contextID)
{
    update();

    render3D(contextID);
}




void AlignerToolRendererMT::renderWarped(unsigned int contextID, bool matchingColors)
{
    updateWarped();

    glDisable(GL_DEPTH_TEST);

    render(contextID, mRadius, matchingColors);

    glEnable(GL_DEPTH_TEST);
}




void AlignerToolRendererMT::renderWarped3D(unsigned int contextID)
{
    updateWarped();

    render3D(contextID);
}




void AlignerToolRendererMT::set(const HxSpatialGraph* microtubules, const Matching* matching, int matchingSet)
{
    mMT          = microtubules;
    mMatching    = matching;
    mMatchingSet = matchingSet;

    mUpdate = true;
}




void AlignerToolRendererMT::setClipSphere(const McVec4f& sphere)
{
    mSphere = sphere;
}




void AlignerToolRendererMT::setColor(const McVec3f& color)
{
    mColor = color;
}




void AlignerToolRendererMT::setRadius(float radius)
{
    mRadius = radius;
}




void AlignerToolRendererMT::setMLS(const MovingLeastSquares& mlsInv)
{
    mMLSInv = &mlsInv;
}




void AlignerToolRendererMT::setMLSQuality(int quality)
{
    mMLSQuality = std::max(std::min(quality, 20), 0);
}




void AlignerToolRendererMT::setSize(int size, int reserve)
{

    mPoints.remax(reserve, size);
    mPointColors.remax(reserve, size);
    mPointPositions.remax(reserve, size);
    mCylinders.remax(reserve, size);
    mCylinderColors.remax(reserve, size);
    mCylinderMap.remax(reserve, size);
    mCylinderPositions.remax(reserve, size);
}




void AlignerToolRendererMT::update()
{
    if (!mUpdate) return;

    if (!mMT)
    {
        setSize(0, 0);
        return;
    }

    // count points and segments

    int idx       = 0;
    int jdx       = 0;
    int numPoints = 0;
    int numSeg    = 0;
    int numEdges  = mMT->getNumEdges();

    for (int i = 0; i < numEdges; ++i)
    {
        if (mMT->getNumEdgePoints(i) < 2) continue;

        numPoints += mMT->getNumEdgePoints(i);
        numSeg    += mMT->getNumEdgePoints(i) - 1;
    }

    // create colors based on a matching

    McDArray< McVec3f > edgeColors[2];

    computeEdgeColors(numEdges, *mMatching, mMatchingSet, edgeColors);

    // set points

    mPoints.resize(numPoints);
    mPointColors.resize(numPoints);
    mPointPositions.resize(numPoints);

    for (int i = 0; i < numEdges; ++i)
    {
        if (mMT->getNumEdgePoints(i) < 2) continue;

        McVec3f colorStart = edgeColors[0][i];
        McVec3f colorEnd   = edgeColors[1][i];
        float   edgeLength = 0.0;
        float   currLength = 0.0;

        for (int j = 0; j < mMT->getNumEdgePoints(i) - 1; ++j)
        {
            edgeLength += (mMT->getEdgePoint(i, j) -
                           mMT->getEdgePoint(i, j + 1)).length();
        }


        for (int j = 0; j < mMT->getNumEdgePoints(i); ++j)
        {
            float s = (currLength / edgeLength);

            mPoints[idx]         = mMT->getEdgePoint(i, j);
            mPointColors[idx]    = s * edgeColors[1][i] + (1.0 - s) * edgeColors[0][i];
            mPointPositions[idx] = s;

            if (j < mMT->getNumEdgePoints(i) - 1)
            {
                currLength += (mMT->getEdgePoint(i, j) -
                               mMT->getEdgePoint(i, j + 1)).length();
            }

            idx++;
        }
    }

    // set segments

    idx = 0;
    jdx = 0;

    mCylinders.resize(numSeg * 2);
    mCylinderColors.resize(numSeg);
    mCylinderMap.resize(numSeg);
    mCylinderPositions.resize(numSeg);

    for (int i = 0; i < numEdges; ++i)
    {
        if (mMT->getNumEdgePoints(i) < 2) continue;

        for (int j = 0; j < mMT->getNumEdgePoints(i) - 1; ++j)
        {
            mCylinders[idx * 2 + 0] = mMT->getEdgePoint(i, j);
            mCylinders[idx * 2 + 1] = mMT->getEdgePoint(i, j + 1);

            mCylinderColors[idx]    = 0.5 * (mPointColors[jdx] + mPointColors[jdx + 1]);
            mCylinderPositions[idx] = mPointPositions[jdx];
            mCylinderMap[idx]       = i;

            idx++;
            jdx++;
        }

        jdx++;
    }

    mUpdate = false;
}




void AlignerToolRendererMT::updateDynamic()
{
    if (!mMT)
    {
        setSize(0, 0);
        return;
    }

    // count points

    int   numEdges  = mMT->getNumEdges();
    int   numPoints = 0;

    for (int i = 0; i < numEdges; ++i)
    {
        numPoints += mMT->getNumEdgePoints(i);
    }

    // create colors based on a matching

    McDArray< McVec3f > edgeColors[2];

    computeEdgeColors(numEdges, *mMatching, mMatchingSet, edgeColors);

    // create geometry

    setSize(0, numPoints * 2);

    McDArray< int > cylinderIndices(0, numPoints);

    // detect all segments close to slice

    int pointIdx    = 0;
    int cylinderIdx = 0;

    for (int i = 0; i < numEdges; ++i)
    {
        int numEdgePoints = mMT->getNumEdgePoints(i);

        if (numEdgePoints < 2) continue;

        // compute length for coloring

        McVec3f colorStart = edgeColors[0][i];
        McVec3f colorEnd   = edgeColors[1][i];
        float   edgeLength = 0.0;
        float   currLength = 0.0;

        for (int j = 0; j < mMT->getNumEdgePoints(i) - 1; ++j)
        {
            edgeLength += (mMT->getEdgePoint(i, j) -
                           mMT->getEdgePoint(i, j + 1)).length();
        }

        // detect cylinder segments

        McVec2i lastPointID(-1, -1);

        float t0 = 0.0f;
        float t1 = computeSliceDistance(mMT->getEdgePoint(i, 0));

        for (int j = 1; j < numEdgePoints; ++j)
        {
            float prev = currLength / edgeLength;

            currLength += (mMT->getEdgePoint(i, j) -
                           mMT->getEdgePoint(i, j - 1)).length();

            float next = currLength / edgeLength;

            t0 = t1;
            t1 = computeSliceDistance(mMT->getEdgePoint(i, j));

            // add cylinder

            if (t0 * t1 < 0.0 || fabs(t0) < mRadius || fabs(t1) < mRadius)
            {
                // add both points

                if (j - 1 != lastPointID.x)
                {
                    mPoints.push(mMT->getEdgePoint(i, j - 1));
                    mPointColors.push(prev * edgeColors[1][i] + (1.0 - prev) * edgeColors[0][i]);
                    mPointPositions.push(prev);

                    cylinderIndices.push(pointIdx);
                    pointIdx++;
                }
                else
                {
                    cylinderIndices.push(lastPointID.y);
                }

                mPoints.push(mMT->getEdgePoint(i, j));
                mPointColors.push(next * edgeColors[1][i] + (1.0 - next) * edgeColors[0][i]);
                mPointPositions.push(next);
                mCylinderMap.push(i);

                lastPointID.setValue(j, pointIdx);
                pointIdx++;
            }
        }
    }

    // warping

    numPoints = mPoints.size();

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int i = 0; i < numPoints; ++i)
    {
        McVec2d p(mPoints[i].x, mPoints[i].y);
        McVec2d q = mMLSInv->interpolateFast(p, mMLSQuality);

        mPoints[i].setValue(q.x, q.y, mPoints[i].z);
    }

    // set-up cylinders

    for (int i = 0; i < cylinderIndices.size(); ++i)
    {
        mCylinders.push(mPoints[cylinderIndices[i]]);
        mCylinders.push(mPoints[cylinderIndices[i] + 1]);

        mCylinderColors.push(mPointColors[cylinderIndices[i]]);
        mCylinderPositions.push(mPointPositions[cylinderIndices[i]]);
    }
}




void AlignerToolRendererMT::updateMatchings(AlignerToolRendererMT& microtubulesSource)
{
    int numMatching = microtubulesSource.getMatching()->getNumMatches();
    int numPoints   = 0;

    mCylinderMatchings.clear();
    mMatchingColors.clear();

    McDArray< McVec4f > points1(0, 100);
    McDArray< McVec4f > points2(0, 100);

    for (int i = 0; i < numMatching; ++i)
    {
        McVec4i m = microtubulesSource.getMatching()->get(i);

        microtubulesSource.computeIntersectionPoints(points1, m.i, mRadius);
        computeIntersectionPoints(points2, m.k, mRadius);

        numPoints += points1.size();
        numPoints += points2.size();

        if (points1.size() < 1 || points2.size() < 1) continue;

        // transform

        for (int j = 0; j < points1.size(); ++j)
        {
            points1[j].z = points2[0].z;
        }

        for (int j = 0; j < points2.size(); ++j)
        {
            McVec2d p(points2[j].x, points2[j].y);
            McVec2d q = mMLSInv->interpolateFast(p, mMLSQuality);

            points2[j].setValue(q.x, q.y, points2[j].z, points2[j].t);
        }

        McVec3f color = microtubulesSource.getMatching()->getColor(i);

        // select best match

        if (points1.size() == 1 && points2.size() == 1)
        {
            float t = microtubulesSource.computeMatchingTransparency(m.j, m.l, points1[0].t, points2[0].t);

            McVec4f c(color.x, color.y, color.z, t);

            //theMsg->printf("color: %f %f %f %f", c.x, c.y, c.z, c.t);

            mCylinderMatchings.push(McVec3f(points1[0].x, points1[0].y, points1[0].z));
            mCylinderMatchings.push(McVec3f(points2[0].x, points2[0].y, points2[0].z));
            mMatchingColors.push(c);
            mMatchingColors.push(c);

        }
        else
        {
            mCylinderMatchings.push(McVec3f(points1[0].x, points1[0].y, points1[0].z));
            mCylinderMatchings.push(McVec3f(points2[0].x, points2[0].y, points2[0].z));

            float d  = (points1[0] - points2[0]).length2();
            int   n  = mCylinderMatchings.size();
            float v1 = points1[0].t;
            float v2 = points2[0].t;

            for (int j = 0; j < points1.size(); ++j)
            {
                for (int k = 0; k < points2.size(); ++k)
                {
                    float d_jk = (points1[j] - points2[k]).length2();

                    if (d_jk < d)
                    {
                        d = d_jk;

                        mCylinderMatchings[n - 2].setValue(points1[j].x, points1[j].y, points1[j].z);
                        mCylinderMatchings[n - 1].setValue(points2[k].x, points2[k].y, points2[k].z);

                        v1 = points1[j].t;
                        v2 = points2[k].t;
                    }
                }
            }

            float t = microtubulesSource.computeMatchingTransparency(m.j, m.l, v1, v2);

            McVec4f c(color.x, color.y, color.z, t);

            //theMsg->printf("color: %f %f %f %f", c.x, c.y, c.z, c.t);

            mMatchingColors.push(c);
            mMatchingColors.push(c);
        }
    }
}




void AlignerToolRendererMT::updateWarped()
{
    if (!mUpdate) return;

    if (!mMT)
    {
        setSize(0, 0);
        return;
    }

    // count points

    int   numEdges  = mMT->getNumEdges();
    int   numPoints = 0;

    for (int i = 0; i < numEdges; ++i)
    {
        numPoints += mMT->getNumEdgePoints(i);
    }

    // create colors based on a matching

    McDArray< McVec3f > edgeColors[2];

    computeEdgeColors(numEdges, *mMatching, mMatchingSet, edgeColors);

    // create geometry

    setSize(0, numPoints * 2);

    McDArray< int > cylinderIndices(0, numPoints);

    // detect all segments close to slice

    int pointIdx    = 0;
    int cylinderIdx = 0;

    for (int i = 0; i < numEdges; ++i)
    {
        int numEdgePoints = mMT->getNumEdgePoints(i);

        if (numEdgePoints < 2) continue;

        // compute length for coloring

        McVec3f colorStart = edgeColors[0][i];
        McVec3f colorEnd   = edgeColors[1][i];
        float   edgeLength = 0.0;
        float   currLength = 0.0;

        for (int j = 0; j < mMT->getNumEdgePoints(i) - 1; ++j)
        {
            edgeLength += (mMT->getEdgePoint(i, j) -
                           mMT->getEdgePoint(i, j + 1)).length();
        }

        // detect cylinder segments

        McVec2i lastPointID(-1, -1);

        float t0 = 0.0f;
        float t1 = computeSliceDistance(mMT->getEdgePoint(i, 0));

        for (int j = 1; j < numEdgePoints; ++j)
        {
            float prev = currLength / edgeLength;

            currLength += (mMT->getEdgePoint(i, j) -
                           mMT->getEdgePoint(i, j - 1)).length();

            float next = currLength / edgeLength;

            t0 = t1;
            t1 = computeSliceDistance(mMT->getEdgePoint(i, j));

            // add cylinder

            //if (t0 * t1 < 0.0 || fabs(t0) < mRadius || fabs(t1) < mRadius)
            {
                // add both points

                if (j - 1 != lastPointID.x)
                {
                    mPoints.push(mMT->getEdgePoint(i, j - 1));
                    mPointColors.push(prev * edgeColors[1][i] + (1.0 - prev) * edgeColors[0][i]);
                    mPointPositions.push(prev);

                    cylinderIndices.push(pointIdx);
                    pointIdx++;
                }
                else
                {
                    cylinderIndices.push(lastPointID.y);
                }

                mPoints.push(mMT->getEdgePoint(i, j));
                mPointColors.push(next * edgeColors[1][i] + (1.0 - next) * edgeColors[0][i]);
                mPointPositions.push(next);
                mCylinderMap.push(i);

                lastPointID.setValue(j, pointIdx);
                pointIdx++;
            }
        }
    }

    // warping

    numPoints = mPoints.size();

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int i = 0; i < numPoints; ++i)
    {
        McVec2d p(mPoints[i].x, mPoints[i].y);
        McVec2d q = mMLSInv->interpolateFast(p, mMLSQuality);

        mPoints[i].setValue(q.x, q.y, mPoints[i].z);
    }

    // set-up cylinders

    for (int i = 0; i < cylinderIndices.size(); ++i)
    {
        mCylinders.push(mPoints[cylinderIndices[i]]);
        mCylinders.push(mPoints[cylinderIndices[i] + 1]);

        mCylinderColors.push(mPointColors[cylinderIndices[i]]);
        mCylinderPositions.push(mPointPositions[cylinderIndices[i]]);
    }

    mUpdate = false;
}

