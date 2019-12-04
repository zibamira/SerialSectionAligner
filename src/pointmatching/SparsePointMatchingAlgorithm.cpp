////////////////////////////////////////////////////////////////////////////
//
// SparsePointMatchingAlgorithm.cpp
//
// Main Authors: Baum
//
////////////////////////////////////////////////////////////////////////////

#include "SparsePointMatchingAlgorithm.h"

SparsePointMatchingAlgorithm::SparsePointMatchingAlgorithm()
{

}

void 
SparsePointMatchingAlgorithm::computePointMatching(
            const PointRepresentation * pointRep1, 
            const PointRepresentation * pointRep2,
            const Transformation      * pointRep2StartTransform,
            PointMatchingDataStruct   * pointMatching)
{

}

#if 0
double
SparsePointMatchingAlgorithm::computeSparseMatching(const ConfPointSet & pointSet1,
                                         const ConfPointSet & pointSet2,
                                         const McMat4f      * pointSet2Transformation,
                                         const double         distThreshold,
                                         McDArray<int>      & matchedPointIx1,
                                         McDArray<int>      & matchedPointIx2,
                                         const double         scaleRmsd)
{
    this->scaleRmsd = scaleRmsd;

    int numPoints1 = pointSet1.getNumPoints();
    int numPoints2 = pointSet2.getNumPoints();

    scoreDivisor = 
        MatchingScore::getScoreDivisor(numPoints1, numPoints2, matchingScoreType);

    buildBipartiteGraph(pointSet1, 
                        pointSet2,
                        pointSet2Transformation,
                        true,          // only create edges that satisfy the distance threshold
                        distThreshold, // distance threshold is not important
                        false);        // don't be greedy, i.e., we also need edges to s and t
    
    assignOutgoingEdgesToVertices();
    
    return computeMatching(numPoints1, matchedPointIx1, matchedPointIx2);
}
#endif
