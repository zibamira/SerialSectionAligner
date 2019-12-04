#include <hxspindle/AlignerToolRendererSlice.h>
#include <hxspindle/SpindleTools.h>

#include <hxcore/HxMessage.h>
#include <hxcore/HxResource.h>
#include <hxcore/HxController.h>
#include <hxcore/HxViewer.h>

#include <hxfield/HxLattice3.h>
#include <hxfield/HxLocation3.h>
#include <hxfield/HxUniformCoord3.h>

#include <hxfield/HxFieldEvaluator.h>


#include <mclib/McVec3.h>
#include <mclib/internal/McWatch.h>
#include <mclib/McRot.h>

#ifdef _OPENMP
#   include <omp.h>
#endif

#include <cmath>


int AlignerToolRendererSlice::mLandmarkSelected = -1;




AlignerToolRendererSlice::Transformation::Transformation()
    : mEnabled(false)
    , mRotationAngle(0.0)
    , mRotationPoint(0.0, 0.0)
    , mTranslation(0.0, 0.0)
{
}




void AlignerToolRendererSlice::Transformation::bind() const
{
    if (!mEnabled) return;

    glPushMatrix();
    glTranslated(mTranslation.x, mTranslation.y, 0.0);
    glTranslated(mRotationPoint.x, mRotationPoint.y, 0.0);
    glRotated(mRotationAngle, 0.0, 0.0, 1.0);
    glTranslated(-mRotationPoint.x, -mRotationPoint.y, 0.0);
}




void AlignerToolRendererSlice::Transformation::compute(const McBox3f& box, const MovingLeastSquares& mls)
{
    McDArray< McVec2d > points(2);
    McDArray< McVec2d > pointsWarped(2);

    points[0].setValue(0.5 * (box[0] + box[1]), 0.5 * (box[2] + box[3]));
    points[1].setValue(0.5 * (box[0] + box[1]), box[3]);

    for (int i = 0; i < 2; i++)
    {
        pointsWarped[i] = mls.interpolate(points[i]);
    }

    mTranslation   = points[0] - pointsWarped[0];
    mRotationPoint = pointsWarped[0];

    // compute rotation angle

    McVec2d upDir  = points[1]       -  points[0];
    McVec2d upDirW = pointsWarped[1] - pointsWarped[0];

    mRotationAngle = upDirW.angle(upDir);

    if (upDirW.x * upDir.y - upDirW.y * upDir.x < 0.0)
    {
        mRotationAngle *= -1.0;
    }

    mRotationAngle *= 180 / 3.14159265;
}




void AlignerToolRendererSlice::Transformation::disable()
{
    mEnabled = false;
}




void AlignerToolRendererSlice::Transformation::enable()
{
    mEnabled = true;
}




bool AlignerToolRendererSlice::Transformation::isEnabled() const
{
    return mEnabled;
}




void AlignerToolRendererSlice::Transformation::set(const McVec2d& rotationPoint, const float rotationAngle, const McVec2d& translation)
{
    mRotationAngle = rotationAngle;
    mRotationPoint = rotationPoint;
    mTranslation   = translation;
}




McVec2f AlignerToolRendererSlice::Transformation::transform(const McVec2f& point) const
{
    return SpindleTools::vec2(transform(McVec3f(point.x, point.y, 0.0f)));
}




McVec2f AlignerToolRendererSlice::Transformation::transformInv(const McVec2f& point) const
{
    return SpindleTools::vec2(transformInv(McVec3f(point.x, point.y, 0.0f)));
}




McVec3f AlignerToolRendererSlice::Transformation::transform(const McVec3f& point) const
{
    McVec3f    pointTrans = point;
    McVec3f    pointRot;
    McRotation rotation(McVec3f(0.0, 0.0, 1.0), mRotationAngle * (M_PI / 180.0));

    pointTrans -= McVec3f((float) mRotationPoint.x, (float) mRotationPoint.y, 0.0);

    rotation.multVec(pointTrans, pointRot);

    pointTrans  = pointRot;
    pointTrans += McVec3f((float) mRotationPoint.x, (float) mRotationPoint.y, 0.0);
    pointTrans += McVec3f((float) mTranslation.x,   (float) mTranslation.y, 0.0);

    return pointTrans;
}




McVec3f AlignerToolRendererSlice::Transformation::transformInv(const McVec3f& point) const
{
    McVec3f    pointTrans = point;
    McVec3f    pointRot;
    McRotation rotation(McVec3f(0.0, 0.0, 1.0), -mRotationAngle * (M_PI / 180.0));

    pointTrans -= McVec3f((float) mTranslation.x,   (float) mTranslation.y, 0.0);
    pointTrans -= McVec3f((float) mRotationPoint.x, (float) mRotationPoint.y, 0.0);

    rotation.multVec(pointTrans, pointRot);

    pointTrans  = pointRot;
    pointTrans += McVec3f((float) mRotationPoint.x, (float) mRotationPoint.y, 0.0);

    return pointTrans;
}




void AlignerToolRendererSlice::Transformation::unbind() const
{
    if (!mEnabled) return;

    glPopMatrix();
}




AlignerToolRendererSlice::AlignerToolRendererSlice(void)
    : AlignerToolRenderer()
    , m3DRadius(2500.0f)
    , mEndPointDensitiesVisible(true)
    , mInteractionControl(CONTROL_NONE)
    , mLandmarkRadius(1.0)
    , mLandmarkScale(1.0)
    , mMLS(0)
    , mMLSNeighbors(3)
    , mMatchingSelected(-1, -1, -1, -1)
    , mRenderMode(ALIGNMENT)
    , mRotate(0.0f, 0.0f)
    , mSelectedMTBottom(-1, -1)
    , mShowWarpingGrid(false)
    , mSliceResolution(1)
{
    mFields[0] = 0;
    mFields[1] = 0;

    mSliceZ[0] = 0.8;
    mSliceZ[1] = 0.2;

    // set default landmark colors

    mLandmarkColors[0].setValue(0.7, 0.1, 0.1);
    //mLandmarkColors[1].setValue(1.0, 0.5, 0.1);
    mLandmarkColors[1].setValue(0.9, 0.9, 0.7);
    mLandmarkColors[2].setValue(0.9, 0.9, 0.7);

    // load all shaders

    mShaderSlicePositions.load("/share/shaders/hxspindle/SlicePositions");
    mShaderLandmarks.load("/share/shaders/hxspindle/Landmarks", 4, GL_POINTS, GL_TRIANGLE_STRIP);
    mShaderAntialiasing.load("/share/shaders/hxspindle/Antialiasing");
    mShaderDeferred3D.load("/share/shaders/hxspindle/Deferred");

    for (int i = 0; i < 16; ++i) mRotationMat[i] = 0.0;

    mRotationMat[0]  = 1.0;
    mRotationMat[5]  = 1.0;
    mRotationMat[10] = 1.0;
    mRotationMat[15] = 1.0;

}





bool AlignerToolRendererSlice::computeRelativeSlicePosition(const SbVec2f& toolPosition, SbVec3f& slicePosition) const
{
    // ray computation (ortho)

    SbMatrixd modelViewProjection    = mMatrixModelView;
              modelViewProjection.multRight(mMatrixProjection);
    SbMatrixd modelViewProjectionInv = modelViewProjection.inverse();

    SbVec3d rayStart(0.0, 0.0, 0.0);
    SbVec3d rayDirection(0.0, 0.0, 0.0);

    modelViewProjectionInv.multVecMatrix(SbVec3d(toolPosition[0], toolPosition[1], 1.0), rayStart);
    modelViewProjectionInv.multDirMatrix(SbVec3d(0.0, 0.0, -1.0), rayDirection);

    rayDirection.normalize();

    // ray plane intersection

    SbVec3d n(0.0, 0.0, 1.0);
    float   w = getZ(0);

    float   t = (w - n.dot(rayStart)) / n.dot(rayDirection);
    SbVec3d p = rayStart + t * rayDirection;

    slicePosition.setValue(p);

    return true;
}




bool AlignerToolRendererSlice::computeSlicePosition(const SbVec2s& viewerPosition, SbVec3f& slicePosition) const
{
    SbVec2f toolPosition;

    if (!windowToToolCoord(viewerPosition, toolPosition))
    {
        return false;
    }

    if (!computeRelativeSlicePosition(toolPosition, slicePosition))
    {
        return false;
    }

    // apply transformation

    if (!mTransformation.isEnabled())
    {
        return true;
    }

    McVec3f slicePositionTrans = mTransformation.transformInv(McVec3f(slicePosition));

    slicePosition = slicePositionTrans;

    return true;
}




bool AlignerToolRendererSlice::computeSlicePosition(const SbVec2s& viewerPosition, McVec2f& slicePosition) const
{
    SbVec3f position3D;

    if (computeSlicePosition(viewerPosition, position3D))
    {
        slicePosition = SpindleTools::vec2(McVec3f(position3D));
        return true;
    }

    return false;
}




void AlignerToolRendererSlice::computeSlicePositions(unsigned int contextID, HxUniformScalarField3* field, McVec4f& slice, bool gpu)
{
    if (gpu)
    {
        glDisable(GL_ALPHA_TEST);
        glDisable(GL_BLEND);

        SbVec2s viewerSize(short(mFrameBuffer.getWidth() / mSliceResolution), short(mFrameBuffer.getHeight() / mSliceResolution));

        const McBox3f& box = field->getBoundingBox();

        McVec3f boxMin(box[0], box[2], box[4]);
        McVec3f boxMax(box[1], box[3], box[5]);

        mFrameBuffer.bind(contextID);

        mShaderSlicePositions.enable(contextID);
        mShaderSlicePositions.setParameter4f(contextID, "slice",  &slice[0]);
        mShaderSlicePositions.setParameter3f(contextID, "boxMin", &boxMin[0]);
        mShaderSlicePositions.setParameter3f(contextID, "boxMax", &boxMax[0]);

        glBegin(GL_QUADS);
        {
            glVertex3d(-1.0, -1.0, 0.0);
            glVertex3d( 1.0, -1.0, 0.0);
            glVertex3d( 1.0,  1.0, 0.0);
            glVertex3d(-1.0,  1.0, 0.0);
        }
        glEnd();

        mShaderSlicePositions.disable();

        mSlicePositions.remax(mFrameBuffer.getWidth() * mFrameBuffer.getHeight(), viewerSize[0] * viewerSize[1]);

        glReadPixels(0, 0, viewerSize[0], viewerSize[1], GL_RGBA, GL_FLOAT, (GLvoid*) mSlicePositions.dataPtr());

        mFrameBuffer.unbind(contextID);

        glEnable(GL_BLEND);
        glEnable(GL_ALPHA_TEST);
    }
    else
    {
        const McBox3f& box = field->getBoundingBox();

        McMat4< double > m;
        McMat4< double > p;

        glGetDoublev(GL_MODELVIEW_MATRIX,  (GLdouble*) &m);
        glGetDoublev(GL_PROJECTION_MATRIX, (GLdouble*) &p);

        McMat4< double > m_i = m.inverse();
        McMat4< double > p_i = p.inverse();

        int dimX = mFrameBuffer.getWidth() / mSliceResolution;
        int dimY = mFrameBuffer.getHeight() / mSliceResolution;
        int n    = dimX * dimY;

        mSlicePositions.remax(mFrameBuffer.getWidth() * mFrameBuffer.getHeight(), n);
        mSliceDepths.remax(mFrameBuffer.getWidth() * mFrameBuffer.getHeight(), n);

        // pre-computations

        McVec4< double > viewDir(0.0f, 0.0f, -1.0f, 0.0);
        McVec4< double > viewDirH;

        p_i.multVecMatrix(viewDir,  viewDirH);
        m_i.multVecMatrix(viewDirH, viewDir);

        McVec3d s(slice[0], slice[1], slice[2]);
        McVec3d r_d(viewDir.x, viewDir.y, viewDir.z);

        r_d.normalize();

        float s_d = s.dot(r_d);

        // computation

        #ifdef _OPENMP
            #pragma omp parallel for
        #endif

        for (int x = 0; x < dimX; ++x)
        {
            float xPos = (((float) x + 0.5f) / (float) (dimX - 1)) * 2.0f - 1.0f;

            for (int y = 0; y < dimY; ++y)
            {
                float yPos  = (((float) y + 0.5f) / (float) (dimY - 1)) * 2.0f - 1.0f;

                McVec4< double > q;
                McVec4< double > w;

                q.setValue(xPos, yPos, 0.0f, 1.0);

                p_i.multVecMatrix(q, w);
                m_i.multVecMatrix(w, q);

                McVec3d r_s(q.x, q.y, q.z);

                double  t = (slice[3] - s.dot(r_s)) / s_d;
                McVec3d v = r_s + t * r_d;

                float inside = 1.0f;
                float depth  = 1.0f;

                if (v.x < box[0] || v.x > box[1] ||
                    v.y < box[2] || v.y > box[3] ||
                    v.z < box[4] || v.z > box[5])
                {
                    inside = 0.0f;
                }

                q.setValue(v.x, v.y, v.z, 1.0);
                m.multVecMatrix(q, w);
                p.multVecMatrix(w, q);

                depth = ((q.z / q.t) + 1.0) * 0.5;


                mSliceDepths[y * dimX + x] = depth;
                mSlicePositions[y * dimX + x].setValue(v.x, v.y, v.z, inside);
            }
        }
    }
}




void AlignerToolRendererSlice::computeSlicePositionsCPU(unsigned int contextID, HxUniformScalarField3* field, McVec4f& slice, int mode)
{
    const McBox3f& box = field->getBoundingBox();

    McMat4< double > m;
    McMat4< double > p;

    glGetDoublev(GL_MODELVIEW_MATRIX,  (GLdouble*) &m);
    glGetDoublev(GL_PROJECTION_MATRIX, (GLdouble*) &p);

    McMat4< double > m_i = m.inverse();
    McMat4< double > p_i = p.inverse();

    int dimX = mFrameBuffer.getWidth() / mSliceResolution;
    int dimY = mFrameBuffer.getHeight() / mSliceResolution;
    int n    = dimX * dimY;

    mSlicePositions.remax(mFrameBuffer.getWidth() * mFrameBuffer.getHeight(), n);

    // pre-computations

    McVec4< double > viewDir(0.0f, 0.0f, -1.0f, 0.0);
    McVec4< double > viewDirH;

    p_i.multVecMatrix(viewDir,  viewDirH);
    m_i.multVecMatrix(viewDirH, viewDir);

    McVec3d s(slice[0], slice[1], slice[2]);
    McVec3d r_d(viewDir.x, viewDir.y, viewDir.z);

    r_d.normalize();

    float s_d = s.dot(r_d);

    // select screen region

    int yStart = 0;
    int yEnd   = dimY;

    if (mode == 1) yStart = dimY / 2;
    if (mode == 2) yEnd   = dimY / 2;

    // computation

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int x = 0; x < dimX; ++x)
    {
        float xPos = (((float) x + 0.5f) / (float) (dimX - 1)) * 2.0f - 1.0f;

        for (int y = yStart; y < yEnd; ++y)
        {
            float yPos  = (((float) y + 0.5f) / (float) (dimY - 1)) * 2.0f - 1.0f;

            McVec4< double > p;
            McVec4< double > w;

            p.setValue(xPos, yPos, 0.0f, 1.0);

            p_i.multVecMatrix(p, w);
            m_i.multVecMatrix(w, p);

            McVec3d r_s(p.x, p.y, p.z);

            double  t = (slice[3] - s.dot(r_s)) / s_d;
            McVec3d v = r_s + t * r_d;

            float inside = 1.0f;

            if (v.x < box[0] || v.x > box[1] ||
                v.y < box[2] || v.y > box[3] ||
                v.z < box[4] || v.z > box[5])
            {
                inside = 0.1f;
            }

            mSlicePositions[y * dimX + x].setValue(v.x, v.y, v.z, inside);
        }
    }
}




void AlignerToolRendererSlice::computeSliceValues(HxUniformScalarField3* field, const McVec4f& color, int mode)
{
    if (mode == 0)
    {
        computeSliceValues(field, color);
        return;
    }

    SbVec2s viewerSize(short(mFrameBuffer.getWidth() / mSliceResolution), short(mFrameBuffer.getHeight() / mSliceResolution));
    SbVec2s textureSize(getNextPowerOfTwo(viewerSize[0]), getNextPowerOfTwo(viewerSize[1]));
    SbVec2s textureSizeFull(getNextPowerOfTwo(mFrameBuffer.getWidth()), getNextPowerOfTwo(mFrameBuffer.getHeight()));

    mSliceValues.remax(textureSizeFull[0] * textureSizeFull[1] * 4, textureSize[0] * textureSize[1] * 4);
    mSliceValues.fill(0);

    float valueMin = 0.0f;
    float valueMax = 255.0f;

    field->getHistogram().getRange(valueMin, valueMax, 0.01, 0.99);

    float valueDiff = valueMax - valueMin;

    // detect size

    int yMin = 0;
    int yMax = viewerSize[1];

    if (mode == 1) yMin = viewerSize[1] / 2;
    if (mode == 2) yMax = viewerSize[1] / 2;

    // start computation

    #ifdef _OPENMP
        int m = omp_get_max_threads();
    #else
        int m = 1;
    #endif

    // create locations for all threads

    McDArray < HxLocation3* > locations(m);

    for (int i = 0; i < m; ++i)
    {
        locations[i] = field->createLocation();
    }

    // evaluate slice values

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int x = 0; x < viewerSize[0]; ++x)
    {

        // get thread id

        #ifdef _OPENMP
                int t   = omp_get_thread_num();
        #else
                int t = 0;
        #endif

        for (int y = yMin; y < yMax; ++y)
        {
            int i = y * viewerSize[0] + x;

            // compute index in texture space

            int idx = y * textureSize[0] + x;

            // background: no slice

            if (mSlicePositions[i][3] < 0.5)
            {
                mSliceValues[idx * 4 + 3] = 0;

                continue;
            }

            // compute value

            McVec3f pos(mSlicePositions[i][0], mSlicePositions[i][1], mSlicePositions[i][2]);

            // validate position (otherwise crash in eval())

            if (pos.x != pos.x || pos.y != pos.y || pos.z != pos.z) continue;

            float sliceValue = 0.5f;

            bool inside = locations[t]->set(pos);

            field->eval(*locations[t], &sliceValue);

            sliceValue = ((sliceValue - valueMin) / valueDiff) * 255.0;
            sliceValue = std::max(0.0f, std::min(255.0f, sliceValue));

            mSliceValues[idx * 4 + 0] = (unsigned char) (color[0] * sliceValue);
            mSliceValues[idx * 4 + 1] = (unsigned char) (color[1] * sliceValue);
            mSliceValues[idx * 4 + 2] = (unsigned char) (color[2] * sliceValue);

            if (!inside)
            {
                mSliceValues[idx * 4 + 3] = 0;
            }
            else
            {
                mSliceValues[idx * 4 + 3] = (unsigned char) (color[3] * 255.0);
            }
        }
    }

    // free locations

    for (int i = 0; i < m; ++i)
    {
        delete locations[i];
    }

    // store values in a texture

    mSliceTexture.setFetchMode(GL_LINEAR);
    mSliceTexture.setFormat(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    mSliceTexture.setSize(textureSize[0], textureSize[1]);
    mSliceTexture.setTextureData(mSliceValues.dataPtr());
}




void AlignerToolRendererSlice::computeSliceValues(HxUniformScalarField3* field, const McVec4f& color)
{
    SbVec2s viewerSize(short(mFrameBuffer.getWidth()) / mSliceResolution, short(mFrameBuffer.getHeight() / mSliceResolution));
    SbVec2s textureSize(getNextPowerOfTwo(viewerSize[0]), getNextPowerOfTwo(viewerSize[1]));
    SbVec2s textureSizeFull(getNextPowerOfTwo(mFrameBuffer.getWidth()), getNextPowerOfTwo(mFrameBuffer.getHeight()));

    mSliceValues.remax(textureSizeFull[0] * textureSizeFull[1] * 4, textureSize[0] * textureSize[1] * 4);
    mSliceValues.fill(0);

    mSliceDepthsTex.remax(textureSizeFull[0] * textureSizeFull[1], textureSize[0] * textureSize[1]);
    mSliceDepthsTex.fill(1.0f);

    float valueMin = 0.0f;
    float valueMax = 255.0f;

    field->getHistogram().getRange(valueMin, valueMax, 0.01, 0.99);

    float valueDiff = valueMax - valueMin;

    int n = mSlicePositions.size();

    #ifdef _OPENMP
        int m = omp_get_max_threads();
    #else
        int m = 1;
    #endif

    // create locations for all threads

    McDArray < HxLocation3* > locations(m);

    for (int i = 0; i < m; ++i)
    {
        locations[i] = field->createLocation();
    }

    // evaluate slice values

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int i = 0; i < n; ++i)
    {
        // background: no slice

        if (mSlicePositions[i][3] < 0.5) continue;

        // compute index in texture space

        int x   = i % viewerSize[0];
        int y   = i / viewerSize[0];
        int idx = y * textureSize[0] + x;

        #ifdef _OPENMP
            int t   = omp_get_thread_num();
        #else
            int t = 0;
        #endif

        // compute value

        McVec3f pos(mSlicePositions[i][0], mSlicePositions[i][1], mSlicePositions[i][2]);

        // validate position (otherwise crash in eval())

        if (pos.x != pos.x || pos.y != pos.y || pos.z != pos.z) continue;

        float sliceValue;

        bool inside = locations[t]->set(pos);

        /*int inside = */field->eval(*locations[t], &sliceValue);

        sliceValue = ((sliceValue - valueMin) / valueDiff) * 255.0;
        sliceValue = std::max(0.0f, std::min(255.0f, sliceValue));

        mSliceValues[idx * 4 + 0] = (unsigned char) (color[0] * sliceValue);
        mSliceValues[idx * 4 + 1] = (unsigned char) (color[1] * sliceValue);
        mSliceValues[idx * 4 + 2] = (unsigned char) (color[2] * sliceValue);

        if (!inside)
        {
            mSliceValues[idx * 4 + 3] = 0.0f;
            mSliceDepthsTex[idx] = 1.0;
        }
        else
        {
            mSliceValues[idx * 4 + 3] = (unsigned char) (color[3] * 255.0);
            mSliceDepthsTex[idx] = mSliceDepths[i];
        }
    }

    // free locations

    for (int i = 0; i < m; ++i)
    {
        delete locations[i];
    }

    // store values in a texture

    mSliceTexture.setFetchMode(GL_LINEAR);
    mSliceTexture.setFormat(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    mSliceTexture.setSize(textureSize[0], textureSize[1]);
    mSliceTexture.setTextureData(mSliceValues.dataPtr());

    // store depth values into texture

    mSliceTextureDepths.setFetchMode(GL_LINEAR);
    mSliceTextureDepths.setFormat(GL_R32F, GL_RED, GL_FLOAT);
    mSliceTextureDepths.setSize(textureSize[0], textureSize[1]);
    mSliceTextureDepths.setTextureData(mSliceDepthsTex.dataPtr());
}




void AlignerToolRendererSlice::computeSliceValues(HxUniformScalarField3* field, HxUniformScalarField3* checkField, McDArray< int >& checkEnds, Matching& matching, const McVec4f& color)
{
    SbVec2s viewerSize(short(mFrameBuffer.getWidth()) / mSliceResolution, short(mFrameBuffer.getHeight() / mSliceResolution));
    SbVec2s textureSize(getNextPowerOfTwo(viewerSize[0]), getNextPowerOfTwo(viewerSize[1]));
    SbVec2s textureSizeFull(getNextPowerOfTwo(mFrameBuffer.getWidth()), getNextPowerOfTwo(mFrameBuffer.getHeight()));

    mSliceValues.remax(textureSizeFull[0] * textureSizeFull[1] * 4, textureSize[0] * textureSize[1] * 4);
    mSliceValues.fill(0);

    float valueMin = 0.0f;
    float valueMax = 255.0f;

    field->getHistogram().getRange(valueMin, valueMax, 0.01, 0.99);

    float valueDiff = valueMax - valueMin;

    int n = mSlicePositions.size();

    #ifdef _OPENMP
        int m = omp_get_max_threads();
    #else
        int m = 1;
    #endif

    // create locations for all threads

    McDArray < HxFieldEvaluator* > evalCheck(m);
    McDArray < HxLocation3* >      locations(m);
    McDArray < HxLocation3* >      locationsCheck(m);

    for (int i = 0; i < m; ++i)
    {
        evalCheck[i] = HxEvalNN::createEval(checkField);
        locations[i] = field->createLocation();

        locationsCheck[i] = evalCheck[i]->createLocation();
    }

    // evaluate slice values

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int i = 0; i < n; ++i)
    {
        // background: no slice

        if (mSlicePositions[i][3] < 0.5) continue;

        // compute index in texture space

        int x   = i % viewerSize[0];
        int y   = i / viewerSize[0];
        int idx = y * textureSize[0] + x;

        #ifdef _OPENMP
            int t   = omp_get_thread_num();
        #else
            int t = 0;
        #endif

        // compute value

        McVec3f pos(mSlicePositions[i][0], mSlicePositions[i][1], mSlicePositions[i][2]);

        // validate position (otherwise crash in eval())

        if (pos.x != pos.x || pos.y != pos.y || pos.z != pos.z) continue;

        locations[t]->set(pos);
        locationsCheck[t]->set(pos);

        float        sliceValue = 0.0f;
        unsigned int labelValue = 0;

        int inside = field->eval(*locations[t], &sliceValue);

        evalCheck[t]->evalNative(locationsCheck[t], &labelValue);

        // filaments check

        if (labelValue <= 0) continue;

        int endID = checkEnds[labelValue - 1];
        int mID   = endID / 2;
        int eID   = endID % 2;

        sliceValue = ((sliceValue - valueMin) / valueDiff) * 255.0;
        sliceValue = std::max(0.0f, std::min(255.0f, sliceValue));

        float scale = 1.0f;

        if (matching.getState(mID, eID, 0) == HxSerialSectionStack::UNDEFINED)
        {
            scale = 0.25f;
        }
        else if (matching.getState(mID, eID, 0) == HxSerialSectionStack::MATCHED_AUTOMATIC)
        {
            scale = 0.6f;
        }

        mSliceValues[idx * 4 + 0] = (unsigned char) (scale * sliceValue * color[0]);
        mSliceValues[idx * 4 + 1] = (unsigned char) (scale * sliceValue * color[1]);
        mSliceValues[idx * 4 + 2] = (unsigned char) (scale * sliceValue * color[2]);

        if (inside == 0)
        {
            mSliceValues[idx * 4 + 3] = 0;
        }
        else
        {
            mSliceValues[idx * 4 + 3] = (unsigned char) (color[3] * 255.0);
        }
    }

    // free locations

    for (int i = 0; i < m; ++i)
    {
        delete evalCheck[i];
        delete locations[i];
        delete locationsCheck[i];
    }

    // store values in a texture

    mSliceTexture.setFetchMode(GL_LINEAR);
    mSliceTexture.setFormat(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    mSliceTexture.setSize(textureSize[0], textureSize[1]);
    mSliceTexture.setTextureData(mSliceValues.dataPtr());
}




void AlignerToolRendererSlice::computeSliceVisXY(const float* box, const McVec4< float >& plane, McVec2f& start, McVec2f& end)
{
    McVec3f isecs[4];
    bool    hit[4];

    hit[0] = planeLineSegIsec(plane, McVec3f(box[0], box[2], box[4]), McVec3f(box[1], box[2], box[4]), isecs[0]);
    hit[1] = planeLineSegIsec(plane, McVec3f(box[0], box[3], box[4]), McVec3f(box[1], box[3], box[4]), isecs[1]);
    hit[2] = planeLineSegIsec(plane, McVec3f(box[0], box[2], box[4]), McVec3f(box[0], box[3], box[4]), isecs[2]);
    hit[3] = planeLineSegIsec(plane, McVec3f(box[1], box[2], box[4]), McVec3f(box[1], box[3], box[4]), isecs[3]);

    bool first = true;

    for (int i = 0; i < 4; ++i)
    {
        if (hit[i])
        {
            if (first)
            {
                start = McVec2f(isecs[i].x, isecs[i].y);
                first = false;
            }
            else
            {
                end = McVec2f(isecs[i].x, isecs[i].y);
                break;
            }
        }
    }
}



void AlignerToolRendererSlice::computeTransformation(const McBox3f& box)
{
    if (mInteractionControl != CONTROL_NONE)
    {
        return;
    }

    mTransformation.enable();
    mTransformation.compute(box, *mMLS);
}





bool AlignerToolRendererSlice::computeWarpGrid1D(McDArray< McVec2d >& grid, const int gridSpacing)
{
    int viewerWidth  = mFrameBuffer.getWidth() / mSliceResolution;
    int viewerHeight = mFrameBuffer.getHeight() / mSliceResolution;
    int gridSize     = ((viewerWidth - 1) / gridSpacing) + 2;

    // grid should be at least 2 in view space and 3 overall

    if (gridSize < 3) return false;

    grid.resize(gridSize);

    // fill grid inside view space (only x)

    int idx = (viewerHeight / 2) * viewerWidth;

    for (int i = 0; i < viewerWidth; i += gridSpacing)
    {
        int x = i / gridSpacing;

        grid[x].setValue(mSlicePositions[idx + i][0], mSlicePositions[idx + i][1]);
    }

    // extrapolate in x direction if necessary

    if ((viewerWidth - 1) % gridSpacing != 0)
    {
        grid[gridSize - 1] = grid[gridSize - 2] + (grid[gridSize - 2] - grid[gridSize - 3]);
    }

    return true;
}




bool AlignerToolRendererSlice::computeWarpGrid(McDArray< McVec2d >& grid, SbVec2s& gridSize, const int gridSpacing)
{
    int viewDimX = mFrameBuffer.getWidth()  / mSliceResolution;
    int viewDimY = mFrameBuffer.getHeight() / mSliceResolution;

    gridSize[0] = (viewDimX - 1) / gridSpacing + 1;
    gridSize[1] = (viewDimY - 1) / gridSpacing + 1;

    if ((viewDimX - 1) % gridSpacing != 0) gridSize[0]++;
    if ((viewDimY - 1) % gridSpacing != 0) gridSize[1]++;

    // grid should be at least 2x2 in view space and 3x3 overall

    if (gridSize[0] < 3 || gridSize[1] < 3) return false;

    int numPoints = gridSize[0] * gridSize[1];

    grid.resize(numPoints);

    // fill grid inside view space (only x, y)

    for (int i = 0; i < viewDimX; i += gridSpacing)
    {
        for (int j = 0; j < viewDimY; j += gridSpacing)
        {
            int y    = j / gridSpacing;
            int x    = i / gridSpacing;
            int idxV = j * viewDimX + i;
            int idxG = y * gridSize[0] + x;

            grid[idxG].setValue(mSlicePositions[idxV][0], mSlicePositions[idxV][1]);
        }
    }

    // extrapolate in y direction (if necessary)

    if ((viewDimY - 1) % gridSpacing != 0)
    {
        int gw = gridSize[0];

        // extrapolate corner in x direction if both
        // sides need to be extrapolated
        if ((viewDimX - 1) % gridSpacing != 0) gw--;

        for (int i = 0; i < gw; ++i)
        {
            int idx  = (gridSize[1] - 1) * gridSize[0] + i;
            int idx2 = (gridSize[1] - 2) * gridSize[0] + i;
            int idx3 = (gridSize[1] - 3) * gridSize[0] + i;

            // last row is computed by the previous two

            grid[idx] = grid[idx2] + (grid[idx2] - grid[idx3]);
        }
    }

    // extrapolation in x direction (if necessary)

    if ((viewDimX - 1) % gridSpacing != 0)
    {
        for (int i = 0; i < gridSize[1]; ++i)
        {
            int idx  = i * gridSize[0] + (gridSize[0] - 1);
            int idx2 = i * gridSize[0] + (gridSize[0] - 2);
            int idx3 = i * gridSize[0] + (gridSize[0] - 3);

            // last column is computed by the previous two

            grid[idx] = grid[idx2] + (grid[idx2] - grid[idx3]);
        }
    }

    return true;
}




void AlignerToolRendererSlice::cutSlicePositions(const McVec4f& cutSphere)
{
    int     n  = mSlicePositions.size();
    float   r2 = cutSphere.t * cutSphere.t;
    McVec3f p(cutSphere.x, cutSphere.y, cutSphere.z);

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int i = 0; i < n; ++i)
    {
        McVec3f q(mSlicePositions[i].x, mSlicePositions[i].y, mSlicePositions[i].z);

        if ((p - q).length2() > r2)
        {
            mSlicePositions[i].t = 0.0;
        }
    }
}




int AlignerToolRendererSlice::getIntersectingLandmark(const McVec2f& position, const McDArray< McVec2f >& landmarks) const
{
    int n = landmarks.size();

    for (int i = 0; i < n; ++i)
    {
        float d = (landmarks[i] - position).length();

        if (d < mLandmarkRadius)
        {
            return i;
        }
    }

    return -1;
}




const McVec2i& AlignerToolRendererSlice::getSelectedMT() const
{
    return mSelectedMTBottom;
}




float AlignerToolRendererSlice::getSlice(int target) const
{
    return mSliceZ[target];
}




AlignerToolRendererSlice::Transformation& AlignerToolRendererSlice::getTransformation()
{
    return mTransformation;
}




float AlignerToolRendererSlice::getZ(int target) const
{
    if (!mFields[target]) return 0.0;

    McBox3f box = mFields[target]->getBoundingBox();

    return (1.0 - mSliceZ[target]) * box[4] + (mSliceZ[target] * box[5]);
}




void AlignerToolRendererSlice::hideGrid()
{
    mShowWarpingGrid = false;
}




void AlignerToolRendererSlice::interpolateGrid(McDArray<McVec2d>& grid, const SbVec2s& gridSize, int gridSpacing)
{
    interpolateGrid(grid, gridSize, gridSpacing, false, 0.0f);
}




void AlignerToolRendererSlice::interpolateGrid(McDArray<McVec2d>& grid, const SbVec2s& gridSize, int gridSpacing, bool setZ, float zValue)
{
    int dimX = mFrameBuffer.getWidth() / mSliceResolution;
    int dimY = mFrameBuffer.getHeight() / mSliceResolution;

    const McBox3f& box = mFields[1]->getBoundingBox();

    for (int i = 0; i < dimX; ++i)
    {
        for (int j = 0; j < dimY; ++j)
        {
            int idx = j * dimX + i;

            double sx = double(i % gridSpacing) / double(gridSpacing);
            double sy = double(j % gridSpacing) / double(gridSpacing);

            int x = i / gridSpacing;
            int y = j / gridSpacing;

            if (x >= gridSize[0] - 1)
            {
                x = gridSize[0] - 2;
                sx = 1.0;
            }
            if (y >= gridSize[1] - 1)
            {
                y = gridSize[1] - 2;
                sy = 1.0;
            }

            int c00 =  y      * gridSize[0] + x;
            int c10 =  y      * gridSize[0] + x + 1;
            int c01 = (y + 1) * gridSize[0] + x;
            int c11 = (y + 1) * gridSize[0] + x + 1;

            McVec2d a1 = (1.0 - sx) * grid[c00] + sx * grid[c10];
            McVec2d a2 = (1.0 - sx) * grid[c01] + sx * grid[c11];
            McVec2d b  = (1.0 - sy) * a1 + sy * a2;

            mSlicePositions[idx][0] = b[0];
            mSlicePositions[idx][1] = b[1];

            if (setZ) mSlicePositions[idx][2] = zValue;

            // set last component to 0 for positions outside the data domain

            if (b[0] < box[0] || b[0] > box[1] ||
                b[1] < box[2] || b[1] > box[3])
            {
                mSlicePositions[idx][3] = 0.0f;
            }
            else
            {
                mSlicePositions[idx][3] = 1.0f;
            }
        }
    }
}




bool AlignerToolRendererSlice::landmarkInteraction(SbVec2s mousePos, bool mouseLeft, bool controlPress, SectionInterface& sectionChange, int section)
{
    bool updateLandmarks = false;

    McVec2f slicePos;

    // start a landmark event (new mark or moving)

    if (mInteractionControl == CONTROL_NONE && mouseLeft)
    {
        if (computeSlicePosition(mousePos, slicePos))
        {
            int landmarkID = getIntersectingLandmark(slicePos, sectionChange.mLandmarks.getMarks(section));

            if (landmarkID >= 0)
            {
                if (controlPress)
                {
                    mInteractionControl = CONTROL_DELETE;
                }
                else
                {
                    mInteractionControl  = CONTROL_MOVE;
                    mLandmarkSelected    = landmarkID;
                    mLandmarkMoveDiff    = sectionChange.mLandmarks.getMarks(section)[landmarkID] - slicePos;
                }
            }
            else if (controlPress)
            {
                mInteractionControl = CONTROL_NEW;
            }
        }
        else
        {
            mInteractionControl = CONTROL_MOUSE_DOWN;
        }
    }

    // add new landmark

    if (mInteractionControl == CONTROL_NEW && !mouseLeft)
    {
        if (computeSlicePosition(mousePos, slicePos))
        {
            McVec2f slicePos1(slicePos);
            McVec2f slicePos2(slicePos);

            if (mTransformation.isEnabled())
            {
                slicePos1 = mTransformation.transform(slicePos);
            }
            else
            {
                slicePos2 = mTransformation.transformInv(slicePos);
            }

            sectionChange.mLandmarks.addMark(slicePos1, slicePos2, HxSerialSectionStack::MANUAL);
            updateLandmarks = true;
        }

        mInteractionControl = CONTROL_NONE;
    }

    // move landmark and stop moving

    if (mInteractionControl == CONTROL_MOVE)
    {
        if (computeSlicePosition(mousePos, slicePos) && mouseLeft)
        {
            sectionChange.mLandmarks.setMark(slicePos + mLandmarkMoveDiff, mLandmarkSelected, section);
            updateLandmarks = true;
        }
        else if (!mouseLeft)
        {
            mInteractionControl  = CONTROL_NONE;
            mLandmarkSelected    = -1;
            updateLandmarks      = true;
        }
    }

    if (mInteractionControl == CONTROL_MOVE && !mouseLeft)
    {
        mInteractionControl = CONTROL_NONE;
        mLandmarkSelected   = -1;
        updateLandmarks     = true;
    }

    // delete landmark

    if (mInteractionControl == CONTROL_DELETE && !mouseLeft)
    {
        if (computeSlicePosition(mousePos, slicePos))
        {
            int landmarkID = getIntersectingLandmark(slicePos, sectionChange.mLandmarks.getMarks(section));

            if (landmarkID >= 0 && controlPress)
            {
                sectionChange.mLandmarks.deleteMark(landmarkID);
                updateLandmarks = true;
            }
        }

        mInteractionControl = CONTROL_NONE;
    }

    // allow new interaction

    if (mInteractionControl == CONTROL_MOUSE_DOWN && !mouseLeft)
    {
        mInteractionControl = CONTROL_NONE;
    }


    return updateLandmarks;
}




bool AlignerToolRendererSlice::matchingInteraction(SbVec2s                 mousePos,
                                                   bool                    mouseLeft,
                                                   bool                    mouseMiddle,
                                                   bool                    controlPress,
                                                   SectionInterface&       sectionChange,
                                                   AlignerToolRendererMT&  filaments1,
                                                   AlignerToolRendererMT&  filaments2,
                                                   McVec2f                 slicePositions,
                                                   McVec2f&                translate)
{
    bool    updateMatching = false;
    SbVec3f slicePos;

    if (mouseLeft && mInteractionControl == CONTROL_NONE)
    {
        mInteractionControl = CONTROL_MOUSE_DOWN;
    }

    if (mouseMiddle && mInteractionControl == CONTROL_NONE)
    {
        mInteractionControl = CONTROL_NEW;
    }

    if (mouseLeft && mInteractionControl == CONTROL_NEW)
    {
        mInteractionControl = CONTROL_NONE;
    }

    // allow new interaction

    if (mInteractionControl == CONTROL_MOUSE_DOWN && !mouseLeft)
    {
        if (controlPress)
        {
            if (computeSlicePosition(mousePos, slicePos))
            {
                McVec3f pos1(slicePos[0], slicePos[1], slicePositions.x);
                McVec3f pos2(slicePos[0], slicePos[1], slicePositions.y);

                McVec2i filament1;
                McVec2i filament2;

                float dist1 = filaments1.computeDistance(pos1, filament1);
                float dist2 = filaments2.computeDistance(pos2, filament2);

                if (dist1 < dist2 && dist1 < filaments1.getRadius())
                {
                    mMatchingSelected.i = filament1.x;
                    mMatchingSelected.j = filament1.y;
                }
                else if (dist2 <= dist1 && dist2 < filaments1.getRadius())
                {
                    mMatchingSelected.k = filament2.x;
                    mMatchingSelected.l = filament2.y;
                }
                else
                {
                    mMatchingSelected.setValue(-1, -1, -1, -1);
                }

                // add or remove matching

                int matchingID = filaments1.getMatch(mMatchingSelected);

                if (matchingID >= 0)
                {
                    sectionChange.mMatching.remove(matchingID);
                    mMatchingSelected.setValue(-1, -1, -1, -1);
                    updateMatching  = true;
                }
                else
                {
                    if (mMatchingSelected.i >= 0 && mMatchingSelected.k >= 0)
                    {
                        sectionChange.mMatching.add(McVec2i(mMatchingSelected.i, mMatchingSelected.j), McVec2i(mMatchingSelected.k, mMatchingSelected.l));
                        mMatchingSelected.setValue(-1, -1, -1, -1);
                        updateMatching  = true;
                    }
                }
            }
        }

        mInteractionControl = CONTROL_NONE;
    }


    // jump to position of click

    if (mInteractionControl == CONTROL_NEW && !mouseMiddle)
    {
        if (computeSlicePosition(mousePos, slicePos))
        {
            McVec2i filament1;

            McVec3f center = mFields[0]->getBoundingBox().getCenter();
            McVec3f pos1   = McVec3f(slicePos[0], slicePos[1], slicePositions.x);
            float   dist1  = filaments1.computeDistance(pos1, filament1);

            translate.setValue(center.x - slicePos[0], center.y - slicePos[1]);

            if (dist1 < filaments1.getRadius())
            {
                mSelectedMTBottom = filament1;
            }
            else
            {
                mSelectedMTBottom.setValue(-1, -1);
            }

            updateMatching = true;
        }

        mInteractionControl = CONTROL_NONE;
    }


    return updateMatching;
}




bool AlignerToolRendererSlice::matchingInteraction3D(SbVec2s mousePos, bool mouseLeft, bool mouseMiddle)
{
    if (mouseLeft && !mouseMiddle && mInteractionControl == CONTROL_NONE)
    {
        SbVec3f slicePosition;

        if (computeSlicePosition(mousePos, slicePosition))
        {
            mInteractionControl = CONTROL_ROTATE;

            mMousePos.x = mousePos[0];
            mMousePos.y = mousePos[1];
        }
    }

    if ((!mouseLeft && mInteractionControl == CONTROL_ROTATE) ||
          mouseMiddle)
    {
        mInteractionControl = CONTROL_NONE;
    }

    if (mInteractionControl == CONTROL_ROTATE)
    {
        mRotate.x += 0.5 * float(mMousePos.x - mousePos[0]);
        mRotate.y += 0.5 * float(mMousePos.y - mousePos[1]);

        mMousePos.x = mousePos[0];
        mMousePos.y = mousePos[1];
    }

    return mInteractionControl != CONTROL_NONE;
}



bool AlignerToolRendererSlice::planeLineSegIsec(const McVec4< float >& plane, const McVec3f& start, const McVec3f& end, McVec3f& isec)
{
    McVec3f r_s = start;
    McVec3f r_d = end - start;
    McVec3f n   = McVec3f(plane.x, plane.y, plane.z);

    if (n.dot(r_d) == 0.0f) return false;

    float t = (plane.t - n.dot(r_s)) / n.dot(r_d);

    if (t < 0.0 || t > 1.0) return false;

    isec = r_s + t * r_d;

    return true;
}




void AlignerToolRendererSlice::printMatrix(const double* m) const
{
    theMsg->printf("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
}





void AlignerToolRendererSlice::printMatrix(const McMat4< double >& m) const
{
    theMsg->printf("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1], m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3]);
}




void AlignerToolRendererSlice::render3D(unsigned int contextID, AlignerToolRendererMT& filaments1, AlignerToolRendererMT& filaments2, const float* sliceValues)
{
    if (!mFields[0] || !mFields[1]) return;

    if (!mMLS) return;

    if (!updateFrameBuffer()) return;

    double modelview[16];
    double projection[16];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    float width = 1.0 / projection[0];

    // slice

    const McBox3f& box1 = mFields[0]->getBoundingBox();
    const McBox3f& box2 = mFields[1]->getBoundingBox();

    mBoxMin.setValue(box1[0], box1[2], box1[4]);
    mBoxMax.setValue(box1[1], box1[3], box1[5]);

    float ratio  = (float) mFrameBuffer.getWidth() / (float) mFrameBuffer.getHeight();
    float height = width / ratio;
    float depth  = (mBoxMax - mBoxMin).length();

    McVec3f center;
    McVec4f plane = filaments1.computePlane(mSelectedMTBottom, center);

    McVec3f planeN(plane.x, plane.y, plane.z);
    McVec3f axis(0.0, 1.0, 0.0);
    float   alpha = (planeN.angle(axis) / M_PI) * 180.0;

    center.x = -modelview[12];
    center.y = -modelview[13];

    plane.t = planeN.dot(center);

    if (axis.cross(planeN).z > 0.0) alpha = -alpha;

    // rendering

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    // bottom section

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-ratio * height, ratio * height, -height, height, -depth * 2.0, depth * 2.0);


    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    double bottomView[16];
    double topView[16];

    // setup rotation

    glLoadMatrixd(mRotationMat);
    glRotatef(-mRotate.x, mRotationMat[1], mRotationMat[5], mRotationMat[9]);
    glRotatef( mRotate.y, mRotationMat[0], mRotationMat[4], mRotationMat[8]);
    mRotate.setValue(0.0f, 0.0f);
    glGetDoublev(GL_MODELVIEW_MATRIX, mRotationMat);

    // setup side view and position

    glTranslatef(0.0, -(mBoxMax[2] - center.z), 0.0);
    glRotatef(alpha, 0.0, 1.0, 0.0);
    glRotatef(-90.0, 1.0, 0.0, 0.0);
    glTranslatef(-center.x, -center.y, -center.z);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glGetDoublev(GL_MODELVIEW_MATRIX, bottomView);

    // render bottom filaments

    McVec4f sphereBottom(center.x, center.y, center.z + (mBoxMax[2] - center.z), m3DRadius);

    filaments1.setClipSphere(sphereBottom);

    renderFilaments(contextID, filaments1, 3, false, true, false);

    // render top filaments

    glLoadMatrixd(mRotationMat);
    glTranslatef(0.0, -(mBoxMax[2] - center.z), 0.0);
    glRotatef(alpha, 0.0, 1.0, 0.0);
    glRotatef(-90.0, 1.0, 0.0, 0.0);
    glTranslatef(-center.x, -center.y, -center.z);
    glTranslatef(0.0, 0.0, box1[5] - box2[4]);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glGetDoublev(GL_MODELVIEW_MATRIX, topView);

    McVec4f sphereTop(center.x, center.y, center.z + (mBoxMax[2] - center.z) - (box1[5] - box2[4]), m3DRadius);

    filaments2.setClipSphere(McVec4f(center.x, center.y, center.z + (mBoxMax[2] - center.z) - (box1[5] - box2[4]), m3DRadius));

    renderFilaments(contextID, filaments2, 4, false, true, false); 

    // render bottom slice

    glLoadMatrixd(bottomView);

    computeSlicePositions(contextID, mFields[0], plane, false);

    cutSlicePositions(sphereBottom);

    computeSliceValues(mFields[0], McVec4f(1.0, 0.92, 0.7, 0.7), 0);

    mShaderDeferred3D.enable(contextID);
    mShaderDeferred3D.setTextureUnit(contextID, "image", 0);
    mShaderDeferred3D.setTextureUnit(contextID, "depth", 1);

    float texWidth  = float(mFrameBuffer.getWidth() / mSliceResolution) / float(mSliceTexture.getWidth());
    float texHeight = float(mFrameBuffer.getHeight() / mSliceResolution) / float(mSliceTexture.getHeight());

    renderTextureDepth(mSliceTexture.getTexture(contextID), mSliceTextureDepths.getTexture(contextID), texWidth, texHeight);

    mShaderDeferred3D.disable();

    // render top slice

    glLoadMatrixd(topView);

    mBoxMin.setValue(box2[0], box2[2], box2[4]);
    mBoxMax.setValue(box2[1], box2[3], box2[5]);

    computeSlicePositions(contextID, mFields[1], plane, false);

    // warp cut sphere

    McVec2d sp(sphereTop.x, sphereTop.y);
    McVec2d spw = mMLS->interpolateFast(sp, mMLSNeighbors);

    sphereTop.x = float(spw.x);
    sphereTop.y = float(spw.y);

    // create a grid

    int gridSpacing = 20;

    SbVec2s             gridSize;
    McDArray< McVec2d > grid;
    McDArray< McVec2d > gridWarped;

    if (!computeWarpGrid(grid, gridSize, gridSpacing)) return;

    // warp grid

    warp(grid, gridWarped);

    // interpolate the warped positions for the full view space

    interpolateGrid(gridWarped, gridSize, gridSpacing);

    // cut by sphere

    cutSlicePositions(sphereTop);

    // compute warped image values

    computeSliceValues(mFields[1], McVec4f(0.7, 0.92, 1.0, 0.7), 0);

    // render with depth information

    mShaderDeferred3D.enable(contextID);
    mShaderDeferred3D.setTextureUnit(contextID, "image", 0);
    mShaderDeferred3D.setTextureUnit(contextID, "depth", 1);

    renderTextureDepth(mSliceTexture.getTexture(contextID), mSliceTextureDepths.getTexture(contextID), texWidth, texHeight);

    mShaderDeferred3D.disable();

    // stop rendering

    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glEnable(GL_DEPTH_TEST);

    glPopAttrib();
}




void AlignerToolRendererSlice::renderEndPointDensities(unsigned int contextID)
{
    if (!updateFrameBuffer()) return;

    mFrameBuffer.bind(contextID);

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_BLEND);

    glDisable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glDisable(GL_LIGHTING);

    if (mEndPointDensitiesVisible)
    {
        // render bar chart

        glBegin(GL_QUADS);
            glColor4f(1.0, 1.0, 1.0, 0.75);
            renderQuad(-1.0, -1.0, 2.0, 0.5);
        glEnd();

        float maxDensity = 0.0;

        for (int i = 0; i < mEndPointDensities.size(); ++i)
        {
            maxDensity = std::max(maxDensity, mEndPointDensities[i]);
        }

        float x = -1.0;
        float w =  0.05;
        float s = 0.4 / maxDensity;

        glBegin(GL_LINE_LOOP);
            glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
            renderQuad(-0.99, -0.975, 4 * 1.2 * w + 0.4 * w, 0.45);
        glEnd();

        for (int i = mEndPointDensities.size() - 1; i >= 0; --i)
        {
            if ( (i + 1) % 4 == 0) x += 0.5 * w;

            float a = 0.1 * float((i) % 4);

            glBegin(GL_QUADS);
                glColor4f(1.0, 0.5 + a, a, 1.0);
                renderQuad(x, -0.95, w, s * mEndPointDensities[i]);
            glEnd();

            x += w * 1.2;
        }

        // render button

        glBegin(GL_QUADS);
            glColor4f(0.5, 0.5, 0.5, 0.75);
            renderQuad(0.91, -0.59, 0.08, 0.08);
        glEnd();
        glBegin(GL_TRIANGLES);
            glColor4f(0.8, 0.8, 0.8, 0.75);
            glVertex3f(0.92, -0.52, 0.0);
            glVertex3f(0.98, -0.52, 0.0);
            glVertex3f(0.95, -0.58, 0.0);
        glEnd();
    }
    else
    {
        glBegin(GL_QUADS);
            glColor4f(0.5, 0.5, 0.5, 0.75);
            renderQuad(0.91, -0.99, 0.08, 0.08);
        glEnd();
        glBegin(GL_TRIANGLES);
            glColor4f(0.8, 0.8, 0.8, 0.75);
            glVertex3f(0.92, -0.98, 0.0);
            glVertex3f(0.98, -0.98, 0.0);
            glVertex3f(0.95, -0.92, 0.0);
        glEnd();
    }

    // stop rendering

    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glEnable(GL_DEPTH_TEST);

    glPopAttrib();

    mFrameBuffer.unbind(contextID);

    renderTexture(mFrameBuffer.getImageTexture(contextID));

}




void AlignerToolRendererSlice::renderGrid(      unsigned int         contextID,
                                          const SbVec2s&             gridSize,
                                          const McDArray< McVec2d >& grid,
                                          const McDArray< McVec2d >& gridWarped,
                                          const double               z)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);

    mFrameBuffer.bind(contextID);

    glColor3f(0.0, 0.0, 1.0);

    glBegin(GL_LINES);
    {
        for (int i = 0; i < gridSize[0]; ++i)
        {
            int v1 = i;
            int v2 = (gridSize[1] - 1) * gridSize[0] + i;

            glVertex3d(grid[v1][0], grid[v1][1], z);
            glVertex3d(grid[v2][0], grid[v2][1], z);
        }

        for (int i = 0; i < gridSize[1]; ++i)
        {
            int v1 = i * gridSize[0] + 0;
            int v2 = i * gridSize[0] + (gridSize[0] - 1);

            glVertex3d(grid[v1][0], grid[v1][1], z);
            glVertex3d(grid[v2][0], grid[v2][1], z);
        }
    }
    glEnd();

    // render warped grid

    glColor3f(1.0, 1.0, 0.0);
    glDisable(GL_DEPTH_TEST);
    glBegin(GL_LINES);
    {
        for (int i = 0; i < gridSize[0] - 1; ++i)
        {
            for (int j = 0; j < gridSize[1] - 1; ++j)
            {
                int idxW1 = j * gridSize[0] + i;
                int idxW2 = j * gridSize[0] + i + 1;
                int idxW3 = (j + 1) * gridSize[0] + i;

                glVertex3d(gridWarped[idxW1][0], gridWarped[idxW1][1], z);
                glVertex3d(gridWarped[idxW2][0], gridWarped[idxW2][1], z);

                glVertex3d(gridWarped[idxW1][0], gridWarped[idxW1][1], z);
                glVertex3d(gridWarped[idxW3][0], gridWarped[idxW3][1], z);
            }
        }
    }
    glEnd();
    glEnable(GL_DEPTH_TEST);

    mFrameBuffer.unbind(contextID);
}



void AlignerToolRendererSlice::renderLandmarks(unsigned int contextID, const Landmarks& landmarks, int sectionID)
{
    double projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    float   width = 1.0 / projection[0];
    float   depth = getZ(0);
    McVec4f slice(0.0, 0.0, 1.0, depth);

    mLandmarkRadius = (mLandmarkScale / (float) mFrameBuffer.getWidth()) * width * 20.0;

    if (!updateFrameBuffer()) return;

    // first pass: render landmarks into frame buffer

    mTransformation.bind();

    glClearColor(0.0, 0.0, 0.0, 0.0);


    float window[2] = {float(mFrameBuffer.getWidth()), float(mFrameBuffer.getHeight())};

    mFrameBuffer.bind(contextID);

    mShaderLandmarks.enable(contextID);
    mShaderLandmarks.setParameter1f(contextID, "radius", mLandmarkRadius);
    mShaderLandmarks.setParameter4f(contextID, "slice", &slice[0]);
    mShaderLandmarks.setParameter1f(contextID, "sliceHeight", 1.0);
    mShaderLandmarks.setParameter2f(contextID, "window", &window[0]);

    const int numMarks = landmarks.getNumMarks();


    glEnable(GL_BLEND);
    glBegin(GL_POINTS);


    for (int i = 0; i < numMarks; ++i)
    {
        if (i == mLandmarkSelected)
        {
            glColor3f(1.0, 0.0, 0.0);
        }
        else
        {
            const McVec3f& color = mLandmarkColors[(int) landmarks.getType(i)];

            glColor3f(color.x, color.y, color.z);
        }

        glVertex4f(landmarks.getMarks(sectionID)[i][0], landmarks.getMarks(sectionID)[i][1], depth - i * 0.001, 1.0);
    }

    glEnd();
    glDisable(GL_BLEND);


    mShaderLandmarks.disable();

    mFrameBuffer.unbind(contextID);

    mTransformation.unbind();

    // second pass: render frame buffer

    mShaderAntialiasing.enable(contextID);
    mShaderAntialiasing.setTextureUnit(contextID, "image", 0);
    mShaderAntialiasing.setParameter2f(contextID, "window", window);
    mShaderAntialiasing.setParameter1f(contextID, "width", width);

    renderTexture(mFrameBuffer.getImageTexture(contextID));

    mShaderAntialiasing.disable();
}




void AlignerToolRendererSlice::renderMatchings(unsigned int contextID, AlignerToolRendererMT& filaments1, AlignerToolRendererMT& filaments2)
{
    if (mRenderMode != MATCHING) return;

    if (!updateFrameBuffer()) return;

    glClearColor(0.0, 0.0, 0.0, 0.0);

    mFrameBuffer.bind(contextID);

    filaments2.renderMatchings(contextID, filaments1);

    mFrameBuffer.unbind(contextID);

    // second pass: render frame buffer

    double projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    float width = 1.0 / projection[0];

    float window[2] = {float(mFrameBuffer.getWidth()), float(mFrameBuffer.getHeight())};

    mShaderAntialiasing.enable(contextID);
    mShaderAntialiasing.setTextureUnit(contextID, "image", 0);
    mShaderAntialiasing.setParameter2f(contextID, "window", window);
    mShaderAntialiasing.setParameter1f(contextID, "width", width);

    renderTexture(mFrameBuffer.getImageTexture(contextID));

    mShaderAntialiasing.disable();
}




void AlignerToolRendererSlice::renderFilaments(unsigned int contextID, AlignerToolRendererMT& filaments)
{
    if (mRenderMode == ALIGNMENT)
    {
        renderFilaments(contextID, filaments, 0, false, false);
    }
    else
    {
        renderFilaments(contextID, filaments, 0, true, true);
    }
}




void AlignerToolRendererSlice::renderFilaments(unsigned int contextID, AlignerToolRendererMT& filaments, int mode, bool border, bool matchingColors, bool setSlice)
{
    if (!updateFrameBuffer()) return;

    // prepare: set slice

    if (setSlice)
    {
        int sectionID = mode == 0 ? 0 : 1;

        filaments.mSlice = McVec4f(0.0, 0.0, 1.0, getZ(sectionID));
    }

    // render twice for border

    if (border)
    {
        float r = filaments.getRadius();

        filaments.setRadius(r * 1.5);
        renderFilaments(contextID, filaments, mode, false, false, false);
        filaments.setRadius(r);
    }

    if (mode == 0) mTransformation.bind();

    // first pass: render filaments into frame buffer

    glClearColor(0.0, 0.0, 0.0, 0.0);

    mFrameBuffer.bind(contextID);

    if (mode == 0)
    {
        filaments.render(contextID, matchingColors);
    }
    else if (mode == 1)
    {
        filaments.renderWarped(contextID, matchingColors);
    }
    else if (mode == 2)
    {
        filaments.renderDynamic(contextID, matchingColors);
    }
    else if (mode == 3)
    {
        filaments.renderStatic3D(contextID);
    }
    else if (mode == 4)
    {
        filaments.renderWarped3D(contextID);
    }

    mFrameBuffer.unbind(contextID);

    // second pass: render frame buffer

    double projection[16];
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    float width = 1.0 / projection[0];

    float window[2] = {float(mFrameBuffer.getWidth()), float(mFrameBuffer.getHeight())};

    if (mode < 3)
    {
        mShaderAntialiasing.enable(contextID);
        mShaderAntialiasing.setTextureUnit(contextID, "image", 0);
        mShaderAntialiasing.setParameter2f(contextID, "window", window);
        mShaderAntialiasing.setParameter1f(contextID, "width", width);

        renderTexture(mFrameBuffer.getImageTexture(contextID));

        mShaderAntialiasing.disable();
    }
    else
    {
        mShaderDeferred3D.enable(contextID);
        mShaderDeferred3D.setTextureUnit(contextID, "image", 0);
        mShaderDeferred3D.setTextureUnit(contextID, "depth", 1);

        renderTextureDepth(mFrameBuffer.getImageTexture(contextID), mFrameBuffer.getDepthTexture(contextID));

        mShaderDeferred3D.disable();
    }



    if (mode == 0) mTransformation.unbind();
}




void AlignerToolRendererSlice::renderFilamentsWarped(unsigned int contextID, AlignerToolRendererMT& filaments)
{
    if (mRenderMode == ALIGNMENT)
    {
        renderFilaments(contextID, filaments, 2, false, false);
    }
    else
    {
        renderFilaments(contextID, filaments, 2, true, true);
    }
}




void AlignerToolRendererSlice::renderPlanes(unsigned int contextID, AlignerToolRendererMT& filaments)
{
    if (!mFields[0]) return;

    if (!updateFrameBuffer()) return;

    const McBox3f& box = mFields[0]->getBoundingBox();

    double modelview[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

    McVec3f center;
    McVec4f plane  = filaments.computePlane(mSelectedMTBottom, center);
    McVec4f plane2 = McVec4f(plane.y, -plane.x, plane.z, plane.t);

    McVec3f planeN(plane.x, plane.y, plane.z);
    McVec3f planeN2(plane2.x, plane2.y, plane2.z);

    center.x = -modelview[12];
    center.y = -modelview[13];

    plane.t = planeN.dot(center);
    plane2.t = planeN2.dot(center);

    mFrameBuffer.bind(contextID, false);

    McVec2f p1;
    McVec2f p2;
    McVec2f p3;
    McVec2f p4;

    computeSliceVisXY(box.constBox6(), plane, p1, p2);
    computeSliceVisXY(box.constBox6(), plane2, p3, p4);

    McVec3f col(0.3, 0.3, 0.3);

    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glColor3f(col.x, col.y, col.z);

    glBegin(GL_LINES);
        glVertex3f(p1.x, p1.y, box[5]);
        glVertex3f(p2.x, p2.y, box[5]);
        glVertex3f(p3.x, p3.y, box[5]);
        glVertex3f(p4.x, p4.y, box[5]);
    glEnd();

    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);

    mFrameBuffer.unbind(contextID);

    renderTexture(mFrameBuffer.getImageTexture(contextID));
}




void AlignerToolRendererSlice::renderQuad(float x, float y, float w, float h, float z)
{
    glVertex3d(x,     y,     z);
    glVertex3d(x + w, y,     z);
    glVertex3d(x + w, y + h, z);
    glVertex3d(x,     y + h, z);
}




void AlignerToolRendererSlice::renderSide(unsigned int           contextID,
                                          AlignerToolRendererMT& filaments1,
                                          AlignerToolRendererMT& filaments2,
                                          const float*           sliceValues,
                                          bool                   ortho)
{
    if (!mFields[0] || !mFields[1]) return;

    if (!mMLS) return;

    if (!updateFrameBuffer()) return;

    double modelview[16];
    double projection[16];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);

    float width = 1.0 / projection[0];

    // slice

    const McBox3f& box1 = mFields[0]->getBoundingBox();
    const McBox3f& box2 = mFields[1]->getBoundingBox();

    mBoxMin.setValue(box1[0], box1[2], box1[4]);
    mBoxMax.setValue(box1[1], box1[3], box1[5]);

    float ratio  = (float) mFrameBuffer.getWidth() / (float) mFrameBuffer.getHeight();
    float height = width / ratio;
    float depth  = (mBoxMax - mBoxMin).length();

    McVec3f center;
    McVec4f plane = filaments1.computePlane(mSelectedMTBottom, center);

    if (ortho)
    {
        float tmp =  plane.x;
        plane.x   =  plane.y;
        plane.y   = -tmp;
    }

    McVec3f planeN(plane.x, plane.y, plane.z);
    McVec3f axis(0.0, 1.0, 0.0);
    float   alpha = (planeN.angle(axis) / M_PI) * 180.0;

    center.x = -modelview[12];
    center.y = -modelview[13];

    plane.t = planeN.dot(center);

    if (axis.cross(planeN).z > 0.0) alpha = -alpha;

    // rendering

    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    // bottom section

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-ratio * height, ratio * height, -height, height, -depth * 2.0, depth * 2.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(0.0, -(mBoxMax[2] - center.z), 0.0);
    glRotatef(alpha, 0.0, 1.0, 0.0);
    glRotatef(-90.0, 1.0, 0.0, 0.0);
    glTranslatef(-center.x, -center.y, -center.z);
    glColor4f(1.0, 1.0, 1.0, 1.0);

    computeSlicePositionsCPU(contextID, mFields[0], plane, 2);

    computeSliceValues(mFields[0], McVec4f(1.0, 0.92, 0.7, 1.0), 2);

    float texWidth  = float(mFrameBuffer.getWidth() / mSliceResolution) / float(mSliceTexture.getWidth());
    float texHeight = float(mFrameBuffer.getHeight() / mSliceResolution) / float(mSliceTexture.getHeight());

    renderTexture(mSliceTexture.getTexture(contextID), texWidth, texHeight);

    filaments1.mSlice = plane;

    renderFilaments(contextID, filaments1, 0, false, true, false);

    // slice position

    mFrameBuffer.bind(contextID, false);

    McVec2f p1;
    McVec2f p2;

    computeSliceVisXY(box1.constBox6(), plane, p1, p2);

    float   sv1  = (1.0 - sliceValues[0]) * box1[4] + sliceValues[0] * box1[5];
    McVec3f col1 = filaments1.getColor();

    glColor3f(col1.x, col1.y, col1.z);

    glBegin(GL_LINES);
        glVertex3f(p1.x, p1.y, sv1);
        glVertex3f(p2.x, p2.y, sv1);
    glEnd();

    mFrameBuffer.unbind(contextID);

    renderTexture(mFrameBuffer.getImageTexture(contextID));

    // top section

    glLoadIdentity();
    glTranslatef(0.0, -(mBoxMax[2] - center.z), 0.0);
    glRotatef(alpha, 0.0, 1.0, 0.0);
    glRotatef(-90.0, 1.0, 0.0, 0.0);
    glTranslatef(-center.x, -center.y, -center.z);
    glTranslatef(0.0, 0.0, box1[5] - box2[4]);
    glColor4f(1.0, 1.0, 1.0, 1.0);

    // first pass: compute slice positions

    mBoxMin.setValue(box2[0], box2[2], box2[4]);
    mBoxMax.setValue(box2[1], box2[3], box2[5]);

    computeSlicePositionsCPU(contextID, mFields[1], plane, 1);

    renderSliceWarpedSide(contextID, mFields[1]);

    // render warped density texture

    glEnable(GL_BLEND);

    renderTexture(mSliceTexture.getTexture(contextID), texWidth, texHeight);

    glDisable(GL_BLEND);

    filaments2.mSlice = plane;

    renderFilaments(contextID, filaments2, 1, false, true, false);

    // slice position

    mFrameBuffer.bind(contextID, false);

    McVec2f p3;
    McVec2f p4;

    computeSliceVisXY(box2.constBox6(), plane, p3, p4);

    McVec2d p3w = mMLS->interpolate(McVec2d(p3.x, p3.y));
    McVec2d p4w = mMLS->interpolate(McVec2d(p4.x, p4.y));

    float   sv2  = (1.0 - sliceValues[1]) * box2[4] + sliceValues[1] * box2[5];
    McVec3f col2 = filaments2.getColor();

    glColor3f(col2.x, col2.y, col2.z);

    glBegin(GL_LINES);
        glVertex3f(p3w.x, p3w.y, sv2);
        glVertex3f(p4w.x, p4w.y, sv2);
    glEnd();

    mFrameBuffer.unbind(contextID);

    renderTexture(mFrameBuffer.getImageTexture(contextID));

    // stop rendering

    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glEnable(GL_DEPTH_TEST);

    glPopAttrib();
}




void AlignerToolRendererSlice::renderSlice(unsigned int contextID, const McVec4f& color)
{
    if (!mFields[0]) return;

    if (!updateFrameBuffer()) return;

    /*double modelview[16];
    double projection[16];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);*/

   // printMatrix(modelview);
   // printMatrix(projection);

    // slice



   // glGetDoublev(GL_MODELVIEW_MATRIX,  mMatrixModelView);
   //  glGetDoublev(GL_PROJECTION_MATRIX, mMatrixProjection);

    // slice

    //if (!updateFrameBuffer()) return;

    mTransformation.bind();

    // first pass: compute slice positions

    McVec4f slice(0.0, 0.0, 1.0, getZ(0));

    computeSlicePositions(contextID, mFields[0], slice, false);

    // compute slice data

    computeSliceValues(mFields[0], color, 0);

    // second pass: render density texture

    float texWidth  = float(mFrameBuffer.getWidth() / mSliceResolution) / float(mSliceTexture.getWidth());
    float texHeight = float(mFrameBuffer.getHeight() / mSliceResolution) / float(mSliceTexture.getHeight());

    renderTexture(mSliceTexture.getTexture(contextID), texWidth, texHeight);

    mTransformation.unbind();


}




void AlignerToolRendererSlice::renderSlice(unsigned int contextID, HxUniformScalarField3* checkField, McDArray< int >& checkEnds, Matching& matching, const McVec4f& color)
{
    if (!mFields[0] || !checkField) return;

    // slice

    if (!updateFrameBuffer()) return;

    mTransformation.bind();

    // first pass: compute slice positions

    McVec4f slice(0.0, 0.0, 1.0, getZ(0));

    computeSlicePositions(contextID, mFields[0], slice, false);

    // compute slice data

    computeSliceValues(mFields[0], checkField, checkEnds, matching, color);

    // second pass: render density texture

    float texWidth  = float(mFrameBuffer.getWidth() / mSliceResolution) / float(mSliceTexture.getWidth());
    float texHeight = float(mFrameBuffer.getHeight() / mSliceResolution) / float(mSliceTexture.getHeight());

    renderTexture(mSliceTexture.getTexture(contextID), texWidth, texHeight);

    mTransformation.unbind();
}




void AlignerToolRendererSlice::renderSliceWarped(unsigned int contextID)
{
    if (!mFields[1] || !mMLS) return;

    if (!updateFrameBuffer()) return;

    // create a grid of previous slice positions

    int gridSpacing = 20;

    SbVec2s             gridSize;
    McDArray< McVec2d > grid;
    McDArray< McVec2d > gridWarped;

    if (!computeWarpGrid(grid, gridSize, gridSpacing)) return;

    // warping

    warp(grid, gridWarped);

    // interpolate the warped positions for the full view space

    float depth = getZ(1);

    interpolateGrid(gridWarped, gridSize, gridSpacing, true, depth);

    // render original and warped grid

    if (mShowWarpingGrid)
    {
        renderGrid(contextID, gridSize, grid, gridWarped, depth);
    }

    // compute warped image values

    computeSliceValues(mFields[1], McVec4f(0.0, 165.0 / 255.0, 1.0, 1.0), 0);

    // render warped density texture

    float texWidth  = float(mFrameBuffer.getWidth() / mSliceResolution) / float(mSliceTexture.getWidth());
    float texHeight = float(mFrameBuffer.getHeight() / mSliceResolution) / float(mSliceTexture.getHeight());

    renderTexture(mSliceTexture.getTexture(contextID), true, texWidth, texHeight);

    // render grid from frame buffer

    if (mShowWarpingGrid)
    {
        renderTexture(mFrameBuffer.getImageTexture(contextID));
    }
}





void AlignerToolRendererSlice::renderSliceWarpedSide(unsigned int contextID, HxUniformScalarField3* field2)
{
    if (!field2) return;

    if (!updateFrameBuffer()) return;

    // create a grid of previous slice positions

    int gridSpacing = 20;

    McDArray< McVec2d > grid;
    McDArray< McVec2d > gridWarped;

    if (!computeWarpGrid1D(grid, gridSpacing)) return;

    // warping

    warp(grid, gridWarped);

    // interpolate the warped positions for the top half view space

    for (int i = 0; i < mFrameBuffer.getWidth() / mSliceResolution; ++i)
    {
        float   sx = float(i % gridSpacing) / float(gridSpacing);
        int     gx = i / gridSpacing;
        McVec2d p  = (1.0 - sx) * gridWarped[gx] + sx * gridWarped[gx + 1];

        for (int j = (mFrameBuffer.getHeight() / mSliceResolution) / 2; j < mFrameBuffer.getHeight() / mSliceResolution; ++j)
        {
            int idx = j * (mFrameBuffer.getWidth() / mSliceResolution) + i;

            mSlicePositions[idx][0] = p[0];
            mSlicePositions[idx][1] = p[1];

            // set last component to 0 for positions outside the data domain

            if (p[0] < mBoxMin[0]                    || p[0] > mBoxMax[0] ||
                p[1] < mBoxMin[1]                    || p[1] > mBoxMax[1] ||
                mSlicePositions[idx][2] < mBoxMin[2] || mSlicePositions[idx][2] > mBoxMax[2])
            {
                mSlicePositions[idx][3] = 0.0f;
            }
            else
            {
                mSlicePositions[idx][3] = 1.0f;
            }
        }
    }

    // compute warped image values

    computeSliceValues(field2, McVec4f(0.7, 0.92, 1.0, 1.0), 1);
}




void AlignerToolRendererSlice::set3DRadius(float radius)
{
    m3DRadius = radius;
}




void AlignerToolRendererSlice::setEndPointDensities(const McDArray<float>& densities)
{
    mEndPointDensities = densities;
}




void AlignerToolRendererSlice::setFields(HxUniformScalarField3* fieldConstant, HxUniformScalarField3* fieldWaped)
{
    mFields[0] = fieldConstant;
    mFields[1] = fieldWaped;
}




void AlignerToolRendererSlice::setLandmarkRadius(float radius)
{
    mLandmarkRadius = radius;
}




void AlignerToolRendererSlice::setLandmarkScale(float scale)
{
    mLandmarkScale = std::max(0.1f, std::min(5.0f, scale));
}




void AlignerToolRendererSlice::setMLS(const MovingLeastSquares& mls)
{
    mMLS = &mls;
}



void AlignerToolRendererSlice::setMLSWarpingPoints(int numPoints)
{
    mMLSNeighbors = std::max(3, std::min(20, numPoints));
}




void AlignerToolRendererSlice::setRenderMode(int mode)
{
    if (mode == 0) mRenderMode = ALIGNMENT;
    else           mRenderMode = MATCHING;
}




void AlignerToolRendererSlice::setSelection(const McVec2i& filamentEnd)
{
    mSelectedMTBottom = filamentEnd;
}




void AlignerToolRendererSlice::setSlice(float sliceZ, int target)
{
    mSliceZ[target] = sliceZ;
}




void AlignerToolRendererSlice::setSliceResolution(int resolution)
{
    mSliceResolution = resolution;
}




void AlignerToolRendererSlice::showGrid()
{
    mShowWarpingGrid = true;
}




bool AlignerToolRendererSlice::stateMapInteraction(SbVec2s mousePos, bool mouseLeft, bool mouseMiddle, bool controlPress, SectionInterface& sectionChange, AlignerToolRendererMT& filaments, McVec2f slicePositions, float& slicePosition, HxUniformScalarField3* checkField, McDArray<int>& checkEnds, McVec2f& translate)
{
    bool interact = false;

    // start selection of MT by region

    if (mouseMiddle && !mouseLeft && mInteractionControl == CONTROL_NONE)
    {
        mInteractionControl = CONTROL_NEW;
    }

    // break selection

    if (mouseMiddle && mouseLeft && mInteractionControl == CONTROL_NEW)
    {
        mInteractionControl = CONTROL_NONE;
    }

    // select MT by region

    if (mInteractionControl == CONTROL_NEW && !mouseMiddle)
    {
        SbVec3f slicePos;

        if (computeSlicePosition(mousePos, slicePos))
        {
            // create locations for all threads

            HxFieldEvaluator* evalCheck  = HxEvalNN::createEval(checkField);
            HxLocation3*      location   = evalCheck->createLocation();
            unsigned int      fieldValue = -1;

            int mID = -1;
            int eID = -1;

            if (location->set(McVec3f(slicePos[0], slicePos[1],  slicePos[2])))
            {
                evalCheck->evalNative(location, &fieldValue);

                // look for MT

                if (fieldValue > 0)
                {
                    int endID = checkEnds[fieldValue - 1];
                    mID   = endID / 2;
                    eID   = endID % 2;
                }
            }

            delete evalCheck;
            delete location;

            if (mID >= 0)
            {
                McVec3f p = filaments.mMT->getEdgePoint(mID, 0);

                if (eID == 1) p = filaments.mMT->getEdgePoint(mID, filaments.mMT->getNumEdgePoints(mID) - 1);

                McVec3f center = mFields[0]->getBoundingBox().getCenter();

                translate.setValue(center.x - p.x, center.y - p.y);

                mSelectedMTBottom.setValue(mID, eID);

                slicePosition = (p.z - mFields[0]->getBoundingBox().getMin().z) / (mFields[0]->getBoundingBox().getMax().z - mFields[0]->getBoundingBox().getMin().z);
                slicePosition = std::max(std::min(slicePosition, 1.0f), 0.0f);

                interact = true;
            }
        }

        mInteractionControl = CONTROL_NONE;
    }

    // button interaction

    if (mouseLeft && mInteractionControl == CONTROL_NONE)
    {
        SbVec2f toolPosition;

        if (windowToToolCoord(mousePos, toolPosition))
        {
            if (mEndPointDensitiesVisible &&
                toolPosition[0] >=  0.91 && toolPosition[0] <=  0.99 &&
                toolPosition[1] >= -0.59 && toolPosition[1] <= -0.51)
            {
                mInteractionControl = CONTROL_MINMAX;
            }
            else if (!mEndPointDensitiesVisible &&
                      toolPosition[0] >=  0.91 && toolPosition[0] <=  0.99 &&
                      toolPosition[1] >= -0.99 && toolPosition[1] <= -0.91)
            {
                mInteractionControl = CONTROL_MINMAX;
            }
        }
    }

    if (mInteractionControl == CONTROL_MINMAX)
    {
        interact = true;
    }

    if (!mouseLeft && mInteractionControl == CONTROL_MINMAX)
    {
        SbVec2f toolPosition;

        if (windowToToolCoord(mousePos, toolPosition))
        {
            if (mEndPointDensitiesVisible &&
                toolPosition[0] >=  0.91 && toolPosition[0] <=  0.99 &&
                toolPosition[1] >= -0.59 && toolPosition[1] <= -0.51)
            {
                mEndPointDensitiesVisible = false;
                interact = true;
            }
            else if (!mEndPointDensitiesVisible &&
                      toolPosition[0] >=  0.91 && toolPosition[0] <=  0.99 &&
                      toolPosition[1] >= -0.99 && toolPosition[1] <= -0.91)
            {
                mEndPointDensitiesVisible = true;
                interact = true;
            }
        }

        mInteractionControl = CONTROL_NONE;
    }

    return interact;
}




void AlignerToolRendererSlice::warp(const McDArray<McVec2d>& points, McDArray<McVec2d>& warpedPoints)
{
    warpedPoints.resize(points.size());

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int i = 0; i < points.size(); ++i)
    {
        warpedPoints[i] = mMLS->interpolateFast(points[i], mMLSNeighbors);
    }
}


