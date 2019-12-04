/////////////////////////////////////////////////////////////////
// 
// AngularPointRepresentation3d.cpp
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include <mclib/McDArray.h>
#include <mclib/McVec3.h>
#include <mclib/McMath.h>
#include <mclib/internal/McAlignPointSets.h>
#include <mclib/internal/McAssert.h>

#include "AngularPointRepresentation3d.h"
#include "AngularCacheObject3d.h"
#include "PointMatchingDataStruct.h"
#include "PointMatchingScoringFunction.h"
#include "Transformation.h"

AngularPointRepresentation3d::AngularPointRepresentation3d(const double maxPointDistance,
                                                           const double cliqueDistThreshold,
                                                           const double maxAngleDistForInitMatching,
                                                           const double minAngleForOptMatching,
                                                           const double maxDistForAngle)
{
    transformType = 0;
    mMaxPointDistance = maxPointDistance;
    mMaxPointDistance2 = maxPointDistance * maxPointDistance;
    mCliqueDistThreshold = cliqueDistThreshold;
    mMaxAngleDistForInitMatching = maxAngleDistForInitMatching;
    mMinAngleForOptMatching = minAngleForOptMatching;
    mMaxDistForAngle = maxDistForAngle;
}

int 
AngularPointRepresentation3d::getNumPoints() const
{
    return mCoords.size();
}

void 
AngularPointRepresentation3d::setCoords(const McDArray<McVec3f> & coords)
{
    this->mCoords = coords;
    
    computeBBoxAndDistMatrix();
}

void
AngularPointRepresentation3d::setOrigCoords(const McDArray<McVec3f> & origCoords)
{
    this->mOrigCoords = origCoords;
}

void 
AngularPointRepresentation3d::setDirections(const McDArray<McVec3f> & directions)
{
    this->mDirections = directions;
    
}

const McDArray<McVec3f> & 
AngularPointRepresentation3d::getCoords() const
{
    return mCoords;
}

const McDArray<McVec3f> & 
AngularPointRepresentation3d::getOrigCoords() const
{
    return mOrigCoords;
}

const McDArray<McVec3f> & 
AngularPointRepresentation3d::getDirections() const
{
    return mDirections;
}

const McDArray<McDArray<double> > & 
AngularPointRepresentation3d::getDistMatrix() const
{
    return distMatrix;
}


void 
AngularPointRepresentation3d::computeBBoxAndDistMatrix()
{
    int numPoints = mCoords.size();

    // compute bounding box and resize distMatrix
    bbox.makeEmpty();
    distMatrix.resize(numPoints);
    for ( int i=0; i<numPoints; i++ ) {
        bbox.extendBy(mCoords[i]);
        distMatrix[i].resize(numPoints);
    }

    // compute distance matrix
    maximumDistance = 0.0f;
    for ( int i=0; i<numPoints; i++ ) {
        distMatrix[i][i] = 0.0f;
        for ( int j=i+1; j<numPoints; j++ ) {
            distMatrix[i][j] = distMatrix[j][i] = 
                (mCoords[i] - mCoords[j]).length();
            if ( distMatrix[i][j] > maximumDistance ) 
                maximumDistance = distMatrix[i][j];
        }
    }
    bbox.extendByEps(0.05);
}

void 
AngularPointRepresentation3d::computeTransformation(
        const PointRepresentation     * pointSet2,
        const PointMatchingDataStruct * pointMatching,
        Transformation                * pointSet2Transform) const
{
    const AngularPointRepresentation3d * simplePointSet2 = 
        static_cast<const AngularPointRepresentation3d *>(pointSet2);

    assert(simplePointSet2);

    const McDArray<McVec3f> & coords2 = simplePointSet2->getCoords();
    
    // compute transformation
    mcAlignPointSets(*(McMat4f*)&pointSet2Transform->getTransformation3d(), 
                     &mCoords[0], 
                     &coords2[0], 
                     pointMatching->getRefPoints(), 
                     pointMatching->getQueryPoints(), 
                     pointMatching->getRefPoints().size(), 
                     transformType);
}

void 
AngularPointRepresentation3d::computeCorrespondenceGraphVertices(
        const PointRepresentation * pointSet2, 
        McDArray<McVec2i>                 & corrGraph) const
{
    int numPoints1 = this->getNumPoints();    
    int numPoints2 = pointSet2->getNumPoints();    

    // all points can be matched
    corrGraph.remax(numPoints1*numPoints2);
    for ( int i=0; i<numPoints1; i++ ) {
        for ( int j=0; j<numPoints2; j++ ) {
            corrGraph.append(McVec2i(i,j));
        }
    }
}

void 
AngularPointRepresentation3d::computeCorrespondenceGraphEdges(
        const PointRepresentation * pointSet2, 
        const McDArray<McVec2i>           & corrGraph,
        McDArray<McBitfield>              & connected) const
{
    connected.resize(corrGraph.size());

    for ( int i=0; i<connected.size(); i++ ) {
        connected[i].resize(corrGraph.size());
        connected[i].setAll();
    }
        
    for ( int i=0; i<corrGraph.size(); i++ ) {
        for ( int j=i+1; j<corrGraph.size(); j++ ) {
            if ( !connectByEdge(i, j, pointSet2, corrGraph) ) {
                connected[i].unset(j);
                connected[j].unset(i);
            } 
        }
    }   
}

bool 
AngularPointRepresentation3d::connectByEdge( const int vertex1, 
                                             const int vertex2, 
                                             const PointRepresentation * otherPoints,
                                             const McDArray<McVec2i>   & corrGraph) const
{
    const McDArray<McDArray<double> > & distMatrix1 = this->getDistMatrix();
    const McDArray<McDArray<double> > & distMatrix2 = otherPoints->getDistMatrix();
    bool distanceThresholdOK = !(fabs(distMatrix1[corrGraph[vertex1][0]][corrGraph[vertex2][0]]- 
                                    distMatrix2[corrGraph[vertex1][1]][corrGraph[vertex2][1]])>
                                    mCliqueDistThreshold);
    const AngularPointRepresentation3d* otherPointsA = dynamic_cast<const AngularPointRepresentation3d*>(otherPoints);
    const McDArray<McVec3f> & directions1 = this->getDirections();
    const McDArray<McVec3f> & directions2 = otherPointsA->getDirections();

    double angleDiff1 = directions1[corrGraph[vertex1][0]].dot(directions1[corrGraph[vertex2][0]]);
    double angleDiff2 = directions2[corrGraph[vertex1][1]].dot(directions2[corrGraph[vertex2][1]]);
    angleDiff1 = acos(angleDiff1);
    angleDiff2 = acos(angleDiff2);

    double angleDiffDiff = fabs (angleDiff1-angleDiff2);
    angleDiffDiff = angleDiffDiff/(2.0*M_PI)*360.0;
    
    bool directionDifferenceOK = angleDiffDiff < this->mMaxAngleDistForInitMatching;

    return directionDifferenceOK && distanceThresholdOK;
}

void 
AngularPointRepresentation3d::setScoreDivisor(
        const PointRepresentation    * pointSet2, 
        PointMatchingScoringFunction * scoringFunction) const
{
    const AngularPointRepresentation3d * angularPointSet2 = 
        dynamic_cast<const AngularPointRepresentation3d *>(pointSet2);

    assert(angularPointSet2);

    scoringFunction->setScoreDivisor(this->getNumPoints(), 
                                     angularPointSet2->getNumPoints());
}

CacheObject *
AngularPointRepresentation3d::startExactPointMatching(
        const PointRepresentation * pointRep2,
        const Transformation      * pointRep2Transform) const
{
    const AngularPointRepresentation3d * angularPointRep2 =
        dynamic_cast<const AngularPointRepresentation3d *>(pointRep2);

    assert(angularPointRep2);

    AngularCacheObject3d * cacheObject = new AngularCacheObject3d();
    cacheObject->setCoordsAndDirections(angularPointRep2->getCoords(),
                                        angularPointRep2->getOrigCoords(),
                                        angularPointRep2->getDirections(),
                                        pointRep2Transform->getTransformation3d());

    return cacheObject;
}

CacheObject * 
AngularPointRepresentation3d::startGreedyPointMatching(
        const PointRepresentation * pointSet2,
        const Transformation      * pointSet2StartTransform) const
{
    const AngularPointRepresentation3d * angularPointSet2 = 
        dynamic_cast<const AngularPointRepresentation3d *>(pointSet2);

    assert(angularPointSet2);

    AngularCacheObject3d * cacheObject = new AngularCacheObject3d();
    cacheObject->setCoordsAndDirections(angularPointSet2->getCoords(),
                                        angularPointSet2->getOrigCoords(),
                                        angularPointSet2->getDirections(),
                                        pointSet2StartTransform->getTransformation3d());

    cacheObject->computeDistancesAndSort(mCoords,
                                         mOrigCoords,
                                         mDirections,
                                         mMaxPointDistance,
                                         mMinAngleForOptMatching);

    return cacheObject;
}

bool 
AngularPointRepresentation3d::getNextMatchingPair(
        CacheObject * cacheObject,
        int         & refPointIx,
        int         & queryPointIx,
        double      & dist2) const
{
    AngularCacheObject3d * angularCacheObject = 
        static_cast<AngularCacheObject3d *>(cacheObject);

    return angularCacheObject->getNextMatchingPair(refPointIx, queryPointIx, dist2);    
}

void 
AngularPointRepresentation3d::finishGreedyPointMatching() const
{
    // empty
}

void 
AngularPointRepresentation3d::finishExactPointMatching() const
{
    // empty
}

void
AngularPointRepresentation3d::setTransformType(const int transType) 
{
    mcassert(transType >= 0 && transType <= 2);
    transformType = transType;
}

bool
AngularPointRepresentation3d::canPointsBeMatched(const int pointRep1PointIdx,
						 const PointRepresentation * pointRep2,
						 const int pointRep2PointIdx,
						 CacheObject * cacheObject,
						 double & squaredEdgeWeight) const
{
    AngularCacheObject3d * angularCacheObject =
        static_cast<AngularCacheObject3d *>(cacheObject);

    const McDArray<McVec3f> & coords2 = angularCacheObject->getCoords();
    const McDArray<McVec3f> & dirs2   = angularCacheObject->getDirections();
    squaredEdgeWeight = (mCoords[pointRep1PointIdx]-coords2[pointRep2PointIdx]).length2();

    double angle = mDirections[pointRep1PointIdx].dot(dirs2[pointRep2PointIdx]);
    angle = acos(angle);
    angle = angle/(2*M_PI)*360.0;
    // Britta, why must the angle be larger than minAngleForOptMatching?
    // I would have thought, that the angle should be smaller than
    // a predefined angle?  Or do you assume that the direction vectors 
    // point in opposite directions? [ Daniel ]

    if ( squaredEdgeWeight <= mMaxPointDistance2 &&
	 angle > mMinAngleForOptMatching )
        return true;
    else
        return false;
}

double
AngularPointRepresentation3d::getSquaredEdgeWeight(const int pointRep1PointIdx,
						   const PointRepresentation * pointRep2,
						   const int pointRep2PointIdx,
						   CacheObject * cacheObject) const
{
    AngularCacheObject3d * angularCacheObject =
        static_cast<AngularCacheObject3d *>(cacheObject);

    const McDArray<McVec3f> & coords2 = angularCacheObject->getCoords();
    const double squaredEdgeWeight = (mCoords[pointRep1PointIdx]-coords2[pointRep2PointIdx]).length2();

    return squaredEdgeWeight;
}
