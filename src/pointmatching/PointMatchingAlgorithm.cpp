////////////////////////////////////////////////////////////////////////////
//
// PointMatchingAlgorithm.cpp
//
// Main Authors: Baum
//
////////////////////////////////////////////////////////////////////////////

#include "PointRepresentation.h"
#include "PointMatchingDataStruct.h"
#include "PointMatchingAlgorithm.h"

#define SCORE_EPS 0.01

PointMatchingAlgorithm::PointMatchingAlgorithm()
{
    this->minPointMatchingSize  = 3;
    this->minPointMatchingScore = 0.0f;
    // set default scoring function, which can be overwritten by
    // setScoringFunction(..)
    this->scoringFunction       = new PointMatchingScoringFunction();
}

PointMatchingAlgorithm::~PointMatchingAlgorithm() 
{
}

void  
PointMatchingAlgorithm::setMinPointMatchingSize(
            const int minPointMatchingSize)
{
    // at least 3 points
    this->minPointMatchingSize = 
        (minPointMatchingSize < 3 ? 3 : minPointMatchingSize);
}

void  
PointMatchingAlgorithm::setMinPointMatchingScore(
            const double minPointMatchingScore)
{
    this->minPointMatchingScore = minPointMatchingScore;
}

void  
PointMatchingAlgorithm::setScoringFunction(
            PointMatchingScoringFunction * scoringFunction)
{
    this->scoringFunction = scoringFunction;
}

bool
PointMatchingAlgorithm::acceptPointMatching(
            const PointRepresentation     * pointRep1, 
            const PointRepresentation     * pointRep2,
            const PointMatchingDataStruct * newPointMatching,
            PointMatchingDataStruct       * pointMatching,
            Transformation                * pointRep2Transform)
{
    if ( newPointMatching->getSize() < minPointMatchingSize ) 
    {
        return false; // point matching is too small
    }

    // compute new transformation based on current point matching
    pointRep1->computeTransformation(pointRep2,
                                     newPointMatching,
                                     pointRep2Transform);
    
    double newScore = newPointMatching->getScore();
    double oldScore = pointMatching->getScore();
    
    if (    newScore < minPointMatchingScore 
         || (newScore-oldScore) <= SCORE_EPS ) 
    {
        return false; // score or score improvement is too small
    }
    
    // new point matching has been accepted, so we save it
    *pointMatching = *newPointMatching;

    return true;
}

