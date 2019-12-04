/////////////////////////////////////////////////////////////////
// 
// PointMatchingScoringFunction.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef POINT_MATCHING_SCORING_FUNCTION_H
#define POINT_MATCHING_SCORING_FUNCTION_H

#include <mclib/McDArray.h>
#include <mclib/McHandable.h>
#include <mclib/McVec3.h>

#include "api.h"

class POINTMATCHING_API PointMatchingScoringFunction : public McHandable {
    
public:
    /** Constructor. */
    PointMatchingScoringFunction();
    
    /** Sets the matching score type. */
    void setScoreType(unsigned int type);

    /** Sets the factor by which the rmsd is scaled. */
    void setRMSDScaleFactor(const double factor);

    /** Sets score divisor computed from @c numPoints1 and @c
        numPoints2. */
    void setScoreDivisor(const int numPoints1, 
                         const int numPoints2);
    
    /** Sets score divisor computed from @c numPoints1 and @c
        numPoints2. */
    void setScoreDivisor(const McDArray<int> & numPoints1, 
                         const McDArray<int> & numPoints2);

    /** Sets a flag such that the scoring function can be computed
        iteratively. */
    virtual void startEditing();

    /** Unsets the editing flag. */
    virtual void finishEditing();

    /** Computes the score for a set of squared distances. */
    virtual double computeScore(const McDArray<double> & squaredDists);
    
    /** Computes the score from the current state of the member
        variables and @c newSquaredDist. */
    virtual double computeScore(const double newSquaredDist);
    
    enum ScoreType {
        NORMALIZE_BY_MIN = 0,
        NORMALIZE_BY_AVG = 1
    };

protected:
    /** Compute score divisor from number of points of both point sets. */
    double computeScoreDivisor(const int numPoints1, 
                               const int numPoints2);

    /** Implements the actual scoring function. Computes the score
        from the current state of the member variables. */
    virtual double computeScore();
    
protected:
    bool         editingFlag;
    unsigned int scoreType;

    int          matchingSize;
    double       sumSquaredDists;
    double       rmsdScaleFactor;
    double       scoreDivisor;
};

#endif
