/////////////////////////////////////////////////////////////////
// 
// IterativePointMatching.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef ITERATIVE_POINT_MATCHING_H
#define ITERATIVE_POINT_MATCHING_H

#include "api.h"

class Transformation;
class PointMatchingDataStruct;
class PointRepresentation;
class PointMatchingAlgorithm;

class POINTMATCHING_API IterativePointMatching {

public:
    IterativePointMatching();
    
    bool computePointMatching(PointMatchingAlgorithm  * pointMatchingAlgorithm,
                              PointRepresentation     * pointRep1, 
                              PointRepresentation     * pointRep2,
                              const Transformation    * pointRep2StartTransform,
                              PointMatchingDataStruct * pointMatching,
                              Transformation          * pointRep2Transform);

    // Same as above, but returns statistics information (number of iterations)
    bool computePointMatching(PointMatchingAlgorithm  * pointMatchingAlgorithm,
                              PointRepresentation     * pointRep1, 
                              PointRepresentation     * pointRep2,
                              const Transformation    * pointRep2StartTransform,
                              PointMatchingDataStruct * pointMatching,
                              Transformation          * pointRep2Transform,
                              int&                      numIterations);
    
protected:
    bool acceptPointMatching(const PointRepresentation     * pointRep1, 
                             const PointRepresentation     * pointRep2,
                             const PointMatchingDataStruct * newPointMatching,
                             PointMatchingDataStruct       * pointMatching);

protected:

};

#endif
