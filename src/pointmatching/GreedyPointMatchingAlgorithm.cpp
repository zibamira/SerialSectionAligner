////////////////////////////////////////////////////////////////////////////
//
// GreedyPointMatchingAlgorithm.cpp
//
// Main Authors: Baum
//
////////////////////////////////////////////////////////////////////////////

#include <float.h>

#include "PointRepresentation.h"
#include "PointMatchingDataStruct.h"
#include "PointMatchingAlgorithm.h"
#include "GreedyPointMatchingAlgorithm.h"
#include "CacheObject.h"

#define SCORE_EPS 0.01

GreedyPointMatchingAlgorithm::GreedyPointMatchingAlgorithm() :
    PointMatchingAlgorithm()
{
   
}

void 
GreedyPointMatchingAlgorithm::computePointMatching(
            const PointRepresentation * pointRep1, 
            const PointRepresentation * pointRep2,
            const Transformation      * pointRep2StartTransform,
            PointMatchingDataStruct   * pointMatching)
{
    // clear point matching
    pointMatching->reset();
    
    // initialize scoring function 
    scoringFunction->startEditing();
    pointRep1->setScoreDivisor(pointRep2, scoringFunction);
    
    // pointRep1 is the reference point representation
    McHandle<CacheObject> cacheObject = 
        pointRep1->startGreedyPointMatching(pointRep2, 
                                            pointRep2StartTransform);
    
    int    refPointIx;
    int    queryPointIx;
    int    matchingSize = 0;
    int    bestMatchingSize = 0;
    double dist2;
    double score;
    double bestMatchingScore = 0.0f;
    
    McDArray<int>   refPoints;
    McDArray<int>   queryPoints;

    while ( pointRep1->getNextMatchingPair(cacheObject, refPointIx, queryPointIx, dist2)) 
    {
        // append values
        refPoints.append(refPointIx);
        queryPoints.append(queryPointIx);
        ++matchingSize; 
        
        // compute score from list of squared distances
        score = scoringFunction->computeScore(dist2);

        // keep track of current best matching
        if (    score > bestMatchingScore 
             && matchingSize >= minPointMatchingSize ) {
            bestMatchingScore = score;
            bestMatchingSize  = matchingSize;
        }
    }

    // thematchingPairDist2.size() best matching is the matching of size bestMatchingSize
    refPoints.resize(bestMatchingSize);
    queryPoints.resize(bestMatchingSize);
    
    // store best matching in pointMatching
    pointMatching->setRefPoints(refPoints);
    pointMatching->setQueryPoints(queryPoints);
    pointMatching->setScore(bestMatchingScore);
    
    // clear temporary data structures
    pointRep1->finishGreedyPointMatching();

    // clean up scoring function 
    scoringFunction->finishEditing();
}
