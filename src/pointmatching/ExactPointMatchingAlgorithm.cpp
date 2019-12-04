////////////////////////////////////////////////////////////////////////////
//
// ExactPointMatchingAlgorithm.cpp
//
// Main Authors: Baum
//
////////////////////////////////////////////////////////////////////////////

#include "PointRepresentation.h"
#include "PointMatchingDataStruct.h"
#include "PointMatchingAlgorithm.h"
#include "ExactPointMatchingAlgorithm.h"
#include "CacheObject.h"
#include <cmath>

ExactPointMatchingAlgorithm::ExactPointMatchingAlgorithm()
{
    m_useUserDefinedEdgeWeights = false;
}


void
ExactPointMatchingAlgorithm::setUserDefinedEdgeWeights(
            const McDArray< McDArray<double> > & edgeWeights)
{
    m_useUserDefinedEdgeWeights = true;
    m_userDefinedEdgeWeights = edgeWeights;
}

void
ExactPointMatchingAlgorithm::computePointMatching(
            const PointRepresentation * pointRep1,
            const PointRepresentation * pointRep2,
            const Transformation      * pointRep2Transform,
            PointMatchingDataStruct   * pointMatching)
{
    // clear point matching
    pointMatching->reset();

    // initialize scoring function
    scoringFunction->startEditing();
    pointRep1->setScoreDivisor(pointRep2, scoringFunction);

    // pointRep1 is the reference point representation
    m_cacheObject =
        pointRep1->startExactPointMatching(pointRep2,
                                           pointRep2Transform);

    createBipartiteGraph(pointRep1, pointRep2);

    McDArray<int> refPoints;
    McDArray<int> queryPoints;
    double bestMatchingScore =
        computeBestMatching(pointRep1, pointRep2, refPoints, queryPoints);

    // store best matching in pointMatching
    pointMatching->setRefPoints(refPoints);
    pointMatching->setQueryPoints(queryPoints);
    pointMatching->setScore(bestMatchingScore);

    // clear temporary data structures
    pointRep1->finishExactPointMatching();

    // clean up scoring function
    scoringFunction->finishEditing();
}

void
ExactPointMatchingAlgorithm::computeMaxCardinalityPointMatching(
            const PointRepresentation * pointRep1,
            const PointRepresentation * pointRep2,
            const Transformation      * pointRep2Transform,
            PointMatchingDataStruct   * pointMatching)
{
    // clear point matching
    pointMatching->reset();

    // initialize scoring function
    scoringFunction->startEditing();
    pointRep1->setScoreDivisor(pointRep2, scoringFunction);

    // pointRep1 is the reference point representation
    m_cacheObject =
        pointRep1->startExactPointMatching(pointRep2,
                                           pointRep2Transform);

    createBipartiteGraph(pointRep1, pointRep2);

    McDArray<int> refPoints;
    McDArray<int> queryPoints;
    double maxCardinalityMatchingScore =
        computeMaxCardinalityMatching(pointRep1, pointRep2, refPoints, queryPoints);

    // store best matching in pointMatching
    pointMatching->setRefPoints(refPoints);
    pointMatching->setQueryPoints(queryPoints);
    pointMatching->setScore(maxCardinalityMatchingScore);

    // clear temporary data structures
    pointRep1->finishExactPointMatching();

    // clean up scoring function
    scoringFunction->finishEditing();
}

double
ExactPointMatchingAlgorithm::computeBestMatching(const PointRepresentation * pointRep1,
						 const PointRepresentation * pointRep2,
						 McDArray<int> & refPoints,
                                                 McDArray<int> & queryPoints)
{
    bool validMatchingFound = false;
    double bestScore = -1.0;
    double newScore = 0.0;
    McDArray<McDijkstraEdge *> bestMatching;
    McDArray<McDijkstraEdge *> newMatching;
    do {
        validMatchingFound = findMatching(newMatching);

	newScore = getScoreOfMatching(pointRep1, pointRep2, newMatching);

        if ( newMatching.size()>=3 && newScore>bestScore )
        {
            bestScore = newScore;
            bestMatching = newMatching;
        }
    } while ( validMatchingFound && newScore > 0.0 );

    getRefAndQueryPointsFromMatching(pointRep1, bestMatching, refPoints, queryPoints);

    return bestScore;
}

double
ExactPointMatchingAlgorithm::computeMaxCardinalityMatching(
    const PointRepresentation * pointRep1,
    const PointRepresentation * pointRep2,
    McDArray<int> & refPoints,
    McDArray<int> & queryPoints)
{
    bool validMatchingFound = false;
    McDArray<McDijkstraEdge *> maxCardMatching;
    do {
        validMatchingFound = findMatching(maxCardMatching);
    } while ( validMatchingFound );

    const double maxCardMatchingScore =
        getScoreOfMatching(pointRep1, pointRep2, maxCardMatching);

    getRefAndQueryPointsFromMatching(pointRep1, maxCardMatching, refPoints, queryPoints);

    return maxCardMatchingScore;
}

double
ExactPointMatchingAlgorithm::getEdgeWeight(const PointRepresentation * pointRep1,
                                           const int                   pointRep1PointIdx,
                                           const PointRepresentation * pointRep2,
                                           const int                   pointRep2PointIdx)
{
    if ( m_useUserDefinedEdgeWeights )
        return m_userDefinedEdgeWeights[pointRep1PointIdx][pointRep2PointIdx];
    else
        return pointRep1->getSquaredEdgeWeight(pointRep1PointIdx, pointRep2, pointRep2PointIdx, m_cacheObject);
}

double
ExactPointMatchingAlgorithm::getScoreOfMatching(const PointRepresentation * pointRep1,
						const PointRepresentation * pointRep2,
					        McDArray<McDijkstraEdge *> matching)
{
    McDArray<int> refPoints;
    McDArray<int> queryPoints;

    getRefAndQueryPointsFromMatching(pointRep1, matching, refPoints, queryPoints);

    McDArray<double> squaredWeights(matching.size());
    for ( int i=0; i<matching.size(); ++i )
    {
        squaredWeights[i] =
            getEdgeWeight(pointRep1, refPoints[i], pointRep2, queryPoints[i]);
    }

    return scoringFunction->computeScore(squaredWeights);
}

void
ExactPointMatchingAlgorithm::getRefAndQueryPointsFromMatching(const PointRepresentation * pointRep1,
							      McDArray<McDijkstraEdge *> matching,
							      McDArray<int> & refPoints,
							      McDArray<int> & queryPoints)
{
    const int numPoints1 = pointRep1->getNumPoints();

    refPoints.resize(matching.size());
    queryPoints.resize(matching.size());
    for ( int i=0; i<matching.size(); ++i )
    {
        if ( matching[i]->endVertex < matching[i]->startVertex )
	{
            refPoints[i]   = matching[i]->endVertex;
            queryPoints[i] = matching[i]->startVertex-numPoints1;
        }
	else
	{
            refPoints[i]   = matching[i]->startVertex;
            queryPoints[i] = matching[i]->endVertex-numPoints1;
        }
    }
}

void
ExactPointMatchingAlgorithm::createBipartiteGraph(const PointRepresentation * pointRep1,
						  const PointRepresentation * pointRep2)
{
    int numPoints1 = pointRep1->getNumPoints();
    int numPoints2 = pointRep2->getNumPoints();

    m_startVertex = numPoints1 + numPoints2;
    m_terminalVertex = m_startVertex + 1;

    m_edges.clear();
    m_edges.resize(numPoints1*numPoints2 + numPoints1 + numPoints2);

    // create edges between points
    int cv=0;
    int ce=0;
    for ( int i=0; i<numPoints1; ++i, ++cv )
    {
        int endVertex = numPoints1;
        for ( int j=0; j<numPoints2; ++j, ++endVertex )
        {
            double squaredEdgeWeight = 0.0;
            if ( !pointRep1->canPointsBeMatched(i, pointRep2, j, m_cacheObject, squaredEdgeWeight) )
                continue;

            m_edges[ce].startVertex = cv;
            m_edges[ce].endVertex   = endVertex;
            if ( m_useUserDefinedEdgeWeights )
                m_edges[ce].edgeLength = m_userDefinedEdgeWeights[i][j];
            else
                m_edges[ce].edgeLength = squaredEdgeWeight;

            ++ce;
        }
    }

    // all points plus vertices s and t
    m_vertices.clear();
    m_vertices.resize(numPoints1 + numPoints2 + 2);

    // edges from starting vertex s to vertices of first point set
    for ( int i=0; i<numPoints1; ++i, ++ce )
    {
        m_edges[ce].startVertex  = m_startVertex;
        m_edges[ce].endVertex    = i;
        m_edges[ce].edgeLength   = 0.0;
    }
        
    // edges from vertices of second point set to end vertex t
    cv = numPoints1;
    for ( int i=0; i<numPoints2; ++i, ++cv, ++ce )
    {
        m_edges[ce].startVertex  = cv;
        m_edges[ce].endVertex    = m_terminalVertex;
        m_edges[ce].edgeLength   = 0.0;
    }

    m_edges.resize(ce);

    for ( int i=0; i<m_edges.size(); ++i )
    {
        m_vertices[m_edges[i].startVertex].edges.append(&m_edges[i]);
    }
}

bool 
ExactPointMatchingAlgorithm::findMatching(McDArray<McDijkstraEdge *> & matching)
{
    McDijkstraShortestPath::computeOneToAll(m_vertices, m_edges, m_startVertex);

    // current edge to be looked at
    McDijkstraEdge * currEdge;

    // modify edge weights
    for ( int i=0; i<m_vertices.size(); ++i ) {
        
        currEdge = m_vertices[i].edges.first();
        while ( currEdge ) {
            currEdge->edgeLength += 
                ( m_vertices[i].distance - 
                  m_vertices[currEdge->endVertex].distance );
            
            if ( currEdge->edgeLength < 0.0 ) {
                if ( std::abs(currEdge->edgeLength) > 0.000001 ) {
                    // edgeLength is too negative
                    printf("Error in ExactPointMatchingAlgorithm::findMatching: "
                           "negative edge length = %f\n", 
                           currEdge->edgeLength);
                }
                currEdge->edgeLength = 0.0;
            }

            currEdge = m_vertices[i].edges.succ(currEdge);
        }
    }
    
    if ( !McDijkstraShortestPath::computeOneToOne(m_vertices, m_edges, m_startVertex, m_terminalVertex) ) {
        return false;
    }

    McDArray<McDijkstraEdge *> shortestPathEdges;
    
    McDijkstraEdge * predEdge = m_vertices[m_terminalVertex].predEdge;
    shortestPathEdges.append(predEdge);
    while ( predEdge->startVertex != m_startVertex ) {
        predEdge = m_vertices[predEdge->startVertex].predEdge;
        shortestPathEdges.append(predEdge);
    }

    // remove first edge of minimal s-t-path
    int last = shortestPathEdges.size()-1;
    m_vertices[shortestPathEdges[last]->startVertex].
        edges.remove(shortestPathEdges[last]);
    
    // remove last edge of minimal s-t-path
    m_vertices[shortestPathEdges[0]->startVertex].
        edges.remove(shortestPathEdges[0]);
    
    // invert all other edges 
    int startVertex, endVertex;
    for ( int i=1; i<last; ++i ) {
        startVertex = shortestPathEdges[i]->startVertex;
        endVertex   = shortestPathEdges[i]->endVertex;
        
        m_vertices[startVertex].edges.remove(shortestPathEdges[i]);
        m_vertices[endVertex].edges.append(shortestPathEdges[i]);
        
        // invert edge
        shortestPathEdges[i]->startVertex = endVertex;
        shortestPathEdges[i]->endVertex   = startVertex;
    }
    
    // remove old matching edges first
    McDArray<McDijkstraEdge *> newMatching;
    for ( int i=0; i<matching.size(); ++i )
    {
        if ( matching[i]->startVertex > matching[i]->endVertex )
            newMatching.append(matching[i]);
    }
    
    // append new matching edges
    for ( int i=1; i<last; i+=2 )
    {
        newMatching.append(shortestPathEdges[i]);
    }
    
    matching = newMatching;

    return true;
}
