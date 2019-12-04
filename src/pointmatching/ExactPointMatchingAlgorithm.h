/////////////////////////////////////////////////////////////////
// 
// ExactPointMatchingAlgorithm.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef EXACT_POINT_MATCHING_ALGORITHM_H
#define EXACT_POINT_MATCHING_ALGORITHM_H

#include <mclib/McBitfield.h>

#include <hxgraphalgorithms/McDijkstraShortestPath.h>

#include "PointMatchingAlgorithm.h"
#include "PointMatchingScoringFunction.h"
#include "CacheObject.h"
#include "api.h"

class Transformation;
class PointMatchingDataStruct;
class PointRepresentation;

class POINTMATCHING_API ExactPointMatchingAlgorithm : public PointMatchingAlgorithm {
    
public:
    ExactPointMatchingAlgorithm();

    void setUserDefinedEdgeWeights(const McDArray< McDArray<double> > & edgeWeights);
    
    void computePointMatching(const PointRepresentation * pointRep1, 
                              const PointRepresentation * pointRep2,
                              const Transformation      * pointRep2Transform,
                              PointMatchingDataStruct   * pointMatching);

    void computeMaxCardinalityPointMatching(const PointRepresentation * pointRep1, 
                                            const PointRepresentation * pointRep2,
                                            const Transformation      * pointRep2Transform,
                                            PointMatchingDataStruct   * pointMatching);

private:
    McDArray<McDijkstraVertex>   m_vertices;
    McDArray<McDijkstraEdge>     m_edges;
    int                          m_startVertex;
    int                          m_terminalVertex;
                                 
    McHandle< CacheObject >      m_cacheObject;
    bool                         m_useUserDefinedEdgeWeights;
    McDArray< McDArray<double> > m_userDefinedEdgeWeights;

protected:
    void createBipartiteGraph(const PointRepresentation * pointRep1,
                              const PointRepresentation * pointRep2);

    double computeBestMatching(const PointRepresentation * pointRep1,
                               const PointRepresentation * pointRep2,
                               McDArray<int> & refPoints,
                               McDArray<int> & queryPoints);

    double computeMaxCardinalityMatching(const PointRepresentation * pointRep1,
                                         const PointRepresentation * pointRep2,
                                         McDArray<int> & refPoints,
                                         McDArray<int> & queryPoints);

    double getScoreOfMatching(const PointRepresentation * pointRep1,
                              const PointRepresentation * pointRep2,
                              McDArray<McDijkstraEdge *> matching);

    double getEdgeWeight(const PointRepresentation * pointRep1,
                         const int                   pointRep1PointIdx,
                         const PointRepresentation * pointRep2,
                         const int                   pointRep2PointIdx);

    void getRefAndQueryPointsFromMatching(const PointRepresentation * pointRep1,
					  McDArray<McDijkstraEdge *> matching,
					  McDArray<int> & refPoints,
                                          McDArray<int> & queryPoints);

    bool findMatching(McDArray<McDijkstraEdge *> & matching);
};

#endif
