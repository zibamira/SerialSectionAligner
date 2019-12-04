/////////////////////////////////////////////////////////////////
// 
// PointMatchingAlgorithm.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef POINT_MATCHING_ALGORITHM_H
#define POINT_MATCHING_ALGORITHM_H

#include <mclib/McHandle.h>

#include "PointMatchingScoringFunction.h"
#include "api.h"

class Transformation;
class PointMatchingDataStruct;
class PointRepresentation;

class POINTMATCHING_API PointMatchingAlgorithm {
    
public:
    /** Constructor. */
    PointMatchingAlgorithm();

    /** Destructor */
    virtual ~PointMatchingAlgorithm();
    
    /** Sets the minimum size, i.e. number of matched pairs, a point
        matching should have.
        \param[in] minPointMatchingSize Minimum point matching size.
     */
    void setMinPointMatchingSize (const int   minPointMatchingSize);
    
    /** Sets the minimum matching score.
        \param[in] minPointMatchingScore Minimum point matching score.
     */
    void setMinPointMatchingScore(const double minPointMatchingScore);
    
    /** Sets the scoring function to be used.
        \param[in] scoringFunction Pointer to scoring function.
     */
    void setScoringFunction(PointMatchingScoringFunction * scoringFunction);
    
    /** The goal of this function is to compute the optimal point
        matching w.r.t. the given transformation, i.e. @c
        pointRep2Transform. Depending on the used algorithm, this
        matching might be optimal or only sub-optimal.
        \param[in]  pointRep1          Representation of point set 1.
        \param[in]  pointRep2          Representation of point set 2.
        \param[in]  pointRep2Transform The current transformaion of point set 2.
        \param[out] pointMatching      The point matching to be computed.
     */
    virtual void computePointMatching(const PointRepresentation * pointRep1, 
                                      const PointRepresentation * pointRep2,
                                      const Transformation      * pointRep2Transform,
                                      PointMatchingDataStruct   * pointMatching) = 0;
    
    /** Checks whether @c newPointMatching is accepted as new point
        matching. In order to do so, the optimal transformation of @c
        pointRep2 to @c pointRep1 is computed w.r.t. @c
        newPointMatching. If @c newPointMatching is accepted, @c
        pointMatching will be replaced by @c newPointMatching and @c
        pointRep2Transform will be assigned the optimal
        transformation. If @c newPointMatching is not accepted, @c
        pointMatching and @c pointRep2Transform will not be modified.
        \param[in]     pointRep1          Representation of point set 1.
        \param[in]     pointRep2          Representation of point set 2.
        \param[in]     newPointMatching   The point matching to be checked.
        \param[in,out] pointMatching      The point matching that serves as reference.
        \param[out]    pointRep2Transform The possibly new transformaion.
     */
    virtual bool acceptPointMatching(const PointRepresentation     * pointRep1, 
                                     const PointRepresentation     * pointRep2,
                                     const PointMatchingDataStruct * newPointMatching,
                                     PointMatchingDataStruct       * pointMatching,
                                     Transformation                * pointRep2Transform);
        
protected:
    McHandle<PointMatchingScoringFunction> scoringFunction;
    int    minPointMatchingSize;
    double minPointMatchingScore;
};

#endif
