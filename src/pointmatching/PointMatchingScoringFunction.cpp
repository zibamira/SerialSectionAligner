////////////////////////////////////////////////////////////////////////////
//
// PointMatchingScoringFunction.cpp
//
// Main Authors: Baum
//
////////////////////////////////////////////////////////////////////////////

#include <mclib/McDArray.h>
#include <mclib/internal/McAssert.h>

#include "PointMatchingScoringFunction.h"

#include <cmath>

#define EPSILON 0.01

PointMatchingScoringFunction::PointMatchingScoringFunction()
{
    editingFlag     = false;
    scoreType       = NORMALIZE_BY_MIN;

    rmsdScaleFactor = 1.0f;
    scoreDivisor    = 1.0f;
}

void 
PointMatchingScoringFunction::setRMSDScaleFactor(const double factor)
{
    rmsdScaleFactor = factor;
}

void 
PointMatchingScoringFunction::startEditing()
{
    editingFlag     = true;

    matchingSize    = 0;
    sumSquaredDists = 0.0f;
}

void 
PointMatchingScoringFunction::finishEditing()
{
    editingFlag     = false;
}

void 
PointMatchingScoringFunction::setScoreType(unsigned int type)
{
    scoreType = type;
}

void 
PointMatchingScoringFunction::setScoreDivisor(const int numPoints1, 
                                              const int numPoints2)
{
    scoreDivisor = computeScoreDivisor(numPoints1,
                                       numPoints2);
}

void 
PointMatchingScoringFunction::setScoreDivisor(const McDArray<int> & numPoints1, 
                                              const McDArray<int> & numPoints2)
{
    scoreDivisor = 0.0f;
    
    int i;
    for ( i=0; i<numPoints1.size() && i<numPoints2.size(); i++ ) {
        scoreDivisor += computeScoreDivisor(numPoints1[i],
                                            numPoints2[i]);
    }
    
    if ( scoreDivisor <= 0.0f )
        scoreDivisor = 1.0f;
}

double 
PointMatchingScoringFunction::computeScoreDivisor(const int numPoints1, 
                                                  const int numPoints2)
{
    if ( scoreType == NORMALIZE_BY_MIN ) {
        // take minimal number of points, i.e., maximal size of matching
        int minNumPoints = numPoints1 < numPoints2 ? numPoints1 : numPoints2;
        return double(minNumPoints);
    } else if ( scoreType == NORMALIZE_BY_AVG ) {
        // take average number of points -> small point set are not
        // favored over large ones
        return double(numPoints1 + numPoints2) / 2.0;
    } else {
        // error
        return 1.0f;
    }
}

double
PointMatchingScoringFunction::computeScore()
{
    // compute rmsd of matching
    const double rmsd = 
        sqrt(sumSquaredDists / double(matchingSize));
    
    // compute score value using rmsd value and return it
    return ((double(matchingSize) / scoreDivisor) * exp(-rmsdScaleFactor * rmsd));
}
    
double
PointMatchingScoringFunction::computeScore(const double newSquaredDist)
{
    mcassert(editingFlag);

    sumSquaredDists += newSquaredDist;
    ++matchingSize;
 
    return computeScore();
}

double
PointMatchingScoringFunction::computeScore(const McDArray<double> & squaredDists)
{
    sumSquaredDists = 0.0f;
    matchingSize    = squaredDists.size();
    
    int i;
    for ( i=0; i<matchingSize; i++ ) {
        sumSquaredDists += squaredDists[i];
    }
    
    return computeScore();
}
