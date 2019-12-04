/////////////////////////////////////////////////////////////////
// 
// SimplePointRepresentation3d.cpp
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include <mclib/McDArray.h>
#include <mclib/McVec3.h>
#include <mclib/internal/McAlignPointSets.h>
#include <mclib/internal/McAssert.h>

#include "SimplePointRepresentation3d.h"
#include "SimpleCacheObject3d.h"
#include "PointMatchingDataStruct.h"
#include "PointMatchingScoringFunction.h"
#include "Transformation.h"
#include <cmath>

SimplePointRepresentation3d::SimplePointRepresentation3d(const double maxPointDistance,
                                                         const double cliqueDistThreshold)
{
    transformType = 0;
    mMaxPointDistance = maxPointDistance;
    mMaxPointDistance2 = maxPointDistance * maxPointDistance;
    mCliqueDistThreshold = cliqueDistThreshold;
}

int 
SimplePointRepresentation3d::getNumPoints() const
{
    return coords.size();
}

void 
SimplePointRepresentation3d::setCoords(const McDArray<McVec3f> & coords)
{
    this->coords = coords;
    
    computeBBoxAndDistMatrix();
}

const McDArray<McVec3f> & 
SimplePointRepresentation3d::getCoords() const
{
    return coords;
}

const McDArray<McDArray<double> > & 
SimplePointRepresentation3d::getDistMatrix() const
{
    return distMatrix;
}

void 
SimplePointRepresentation3d::computeBBoxAndDistMatrix()
{
    int numPoints = coords.size();

    // compute bounding box and resize distMatrix
    bbox.makeEmpty();
    distMatrix.resize(numPoints);
    for ( int i=0; i<numPoints; i++ ) {
        bbox.extendBy(coords[i]);
        distMatrix[i].resize(numPoints);
    }

    // compute distance matrix
    maximumDistance = 0.0f;
    for ( int i=0; i<numPoints; i++ ) {
        distMatrix[i][i] = 0.0f;
        for ( int j=i+1; j<numPoints; j++ ) {
            distMatrix[i][j] = distMatrix[j][i] = 
                (coords[i] - coords[j]).length();
            if ( distMatrix[i][j] > maximumDistance ) 
                maximumDistance = distMatrix[i][j];
        }
    }
    bbox.extendByEps(0.05);
}

void 
SimplePointRepresentation3d::computeTransformation(
        const PointRepresentation     * pointSet2,
        const PointMatchingDataStruct * pointMatching,
        Transformation                * pointSet2Transform) const
{
    const SimplePointRepresentation3d * simplePointSet2 = 
        static_cast<const SimplePointRepresentation3d *>(pointSet2);

    assert(simplePointSet2);

    const McDArray<McVec3f> & coords2 = simplePointSet2->getCoords();
    
    // compute transformation
    mcAlignPointSets(*(McMat4f*)&pointSet2Transform->getTransformation3d(), 
                     &coords[0], 
                     &coords2[0], 
                     pointMatching->getRefPoints(), 
                     pointMatching->getQueryPoints(), 
                     pointMatching->getRefPoints().size(), 
                     transformType);
}

void 
SimplePointRepresentation3d::computeCorrespondenceGraphVertices(
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
SimplePointRepresentation3d::computeCorrespondenceGraphEdges(
        const PointRepresentation * pointSet2, 
        const McDArray<McVec2i>           & corrGraph,
        McDArray<McBitfield>              & connected) const
{
    connected.resize(corrGraph.size());

    for ( int i=0; i<connected.size(); i++ ) {
        connected[i].resize(corrGraph.size());
        connected[i].setAll();
    }

    const McDArray<McDArray<double> > & distMatrix1 = this->getDistMatrix();
    const McDArray<McDArray<double> > & distMatrix2 = pointSet2->getDistMatrix();
        
    for ( int i=0; i<corrGraph.size(); i++ ) {
        for ( int j=i+1; j<corrGraph.size(); j++ ) {
            if ( std::abs(distMatrix1[corrGraph[i][0]][corrGraph[j][0]]-
                          distMatrix2[corrGraph[i][1]][corrGraph[j][1]]) >
                 mCliqueDistThreshold ) {
                connected[i].unset(j);
                connected[j].unset(i);
            } 
        }
    }   
}

void 
SimplePointRepresentation3d::setScoreDivisor(
        const PointRepresentation    * pointSet2, 
        PointMatchingScoringFunction * scoringFunction) const
{
    const SimplePointRepresentation3d * simplePointSet2 = 
        dynamic_cast<const SimplePointRepresentation3d *>(pointSet2);

    assert(simplePointSet2);

    scoringFunction->setScoreDivisor(this->getNumPoints(), 
                                     simplePointSet2->getNumPoints());
}

CacheObject *
SimplePointRepresentation3d::startExactPointMatching(
        const PointRepresentation * pointRep2,
        const Transformation      * pointRep2Transform) const
{
    const SimplePointRepresentation3d * simplePointRep2 =
        dynamic_cast<const SimplePointRepresentation3d *>(pointRep2);

    assert(simplePointRep2);

    SimpleCacheObject3d * cacheObject = new SimpleCacheObject3d();
    cacheObject->setCoords(simplePointRep2->getCoords(),
                           pointRep2Transform->getTransformation3d());

    return cacheObject;
}

CacheObject * 
SimplePointRepresentation3d::startGreedyPointMatching(
        const PointRepresentation * pointSet2,
        const Transformation      * pointSet2StartTransform) const
{
    const SimplePointRepresentation3d * simplePointSet2 = 
        dynamic_cast<const SimplePointRepresentation3d *>(pointSet2);

    assert(simplePointSet2);

    SimpleCacheObject3d * cacheObject = new SimpleCacheObject3d();
    cacheObject->setCoords(simplePointSet2->getCoords(),
                           pointSet2StartTransform->getTransformation3d());

    cacheObject->computeDistancesAndSort(coords,
                                         mMaxPointDistance);

    return cacheObject;
}

bool 
SimplePointRepresentation3d::getNextMatchingPair(
        CacheObject * cacheObject,
        int         & refPointIx,
        int         & queryPointIx,
        double      & dist2) const
{
    SimpleCacheObject3d * simpleCacheObject = 
        static_cast<SimpleCacheObject3d *>(cacheObject);

    return simpleCacheObject->getNextMatchingPair(refPointIx, queryPointIx, dist2);    
}

void 
SimplePointRepresentation3d::finishGreedyPointMatching() const
{
    // empty
}

void
SimplePointRepresentation3d::finishExactPointMatching() const
{
    // empty
}

void 
SimplePointRepresentation3d::setTransformType(const int transType) 
{
    mcassert(transType >= 0 && transType <= 2);
    transformType = transType;
}

bool
SimplePointRepresentation3d::canPointsBeMatched(const int pointRep1PointIdx,
						const PointRepresentation * pointRep2,
						const int pointRep2PointIdx,
						CacheObject * cacheObject,
						double & squaredEdgeWeight) const
{
    SimpleCacheObject3d * simpleCacheObject =
        static_cast<SimpleCacheObject3d *>(cacheObject);

    const McDArray<McVec3f> & coords2 = simpleCacheObject->getCoords();
    squaredEdgeWeight = (coords[pointRep1PointIdx]-coords2[pointRep2PointIdx]).length2();

    if ( squaredEdgeWeight <= mMaxPointDistance2 )
        return true;
    else
        return false;
}

double
SimplePointRepresentation3d::getSquaredEdgeWeight(const int pointRep1PointIdx,
						  const PointRepresentation * pointRep2,
						  const int pointRep2PointIdx,
						  CacheObject * cacheObject) const
{
    SimpleCacheObject3d * simpleCacheObject =
        static_cast<SimpleCacheObject3d *>(cacheObject);

    const McDArray<McVec3f> & coords2 = simpleCacheObject->getCoords();
    const double squaredEdgeWeight = (coords[pointRep1PointIdx]-coords2[pointRep2PointIdx]).length2();

    return squaredEdgeWeight;
}
