////////////////////////////////////////////////////////////////////////////
//
// IterativePointMatching.cpp
//
// Main Authors: Baum
//
////////////////////////////////////////////////////////////////////////////

#include <mclib/McHandle.h>

#include "PointRepresentation.h"
#include "PointMatchingAlgorithm.h"
#include "PointMatchingDataStruct.h"
#include "Transformation.h"
#include "IterativePointMatching.h"

#define SCORE_EPS 1e-4

IterativePointMatching::IterativePointMatching()
{
    // empty
}

bool
IterativePointMatching::computePointMatching(PointMatchingAlgorithm  * pointMatchingAlgorithm,
                                             PointRepresentation     * pointRep1, 
                                             PointRepresentation     * pointRep2,
                                             const Transformation    * pointRep2StartTransform,
                                             PointMatchingDataStruct * pointMatching,
                                             Transformation          * pointRep2Transform,
                                             int&                      numIterations)
{
    numIterations = 0;

    if (    !pointMatchingAlgorithm 
         || !pointRep1 
         || !pointRep2 
         || !pointRep2StartTransform 
         || !pointMatching 
         || !pointRep2Transform ) 
        return false;
    
    pointMatching->reset();
    
    // Create temporary duplicate of pointMatching. Note, that
    // duplicate() will create an object of the same type as
    // pointMatching. Thus, we do not have to know the exact type of
    // pointMatching.
    McHandle<PointMatchingDataStruct> newPointMatching = 
        pointMatching->duplicate();
    *pointRep2Transform = *pointRep2StartTransform;


    do {
        pointMatchingAlgorithm->computePointMatching(pointRep1, 
                                                     pointRep2,
                                                     pointRep2Transform,
                                                     newPointMatching);
        ++numIterations;

    } while ( pointMatchingAlgorithm->acceptPointMatching(pointRep1, 
                                                          pointRep2,
                                                          newPointMatching, 
                                                          pointMatching,
                                                          pointRep2Transform) );

    return ( pointMatching->getScore() > 0.0f );
}

bool
IterativePointMatching::computePointMatching(PointMatchingAlgorithm  * pointMatchingAlgorithm,
                                             PointRepresentation     * pointRep1, 
                                             PointRepresentation     * pointRep2,
                                             const Transformation    * pointRep2StartTransform,
                                             PointMatchingDataStruct * pointMatching,
                                             Transformation          * pointRep2Transform)
{
    int numIterations = 0;
    return computePointMatching(pointMatchingAlgorithm,
                                pointRep1,
                                pointRep2,
                                pointRep2StartTransform,
                                pointMatching,
                                pointRep2Transform,
                                numIterations);
}

