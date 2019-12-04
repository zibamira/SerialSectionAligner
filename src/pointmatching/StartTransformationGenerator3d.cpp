/////////////////////////////////////////////////////////////////
// 
// StartTransformationGenerator3d.cpp
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include <mclib/McBitfield.h>
#include <mclib/internal/McAlignPointSets.h>
#include <mclib/McDArray.h>
#include <mclib/McVec3.h>
#include <mclib/internal/McAssert.h>

#include "StartTransformationGenerator3d.h"
#include "PointRepresentation.h"
#include "Transformation.h"
#include "McCliqueDetection.h"

StartTransformationGenerator3d::StartTransformationGenerator3d()
{
    this->minCliqueSize = 3;
    this->maxNumStartTransformations = -1;
    this->transformType = 0;
}

void
StartTransformationGenerator3d::setMinimumCliqueSize(const int size)
{
    this->minCliqueSize = size;
}

void 
StartTransformationGenerator3d::setMaxNumStartTransformations(const int numStartTransformations)
{
    this->maxNumStartTransformations = numStartTransformations;
}


void 
StartTransformationGenerator3d::setTransformType(const int transType) 
{
    mcassert(transType >= 0 && transType <= 2);
    this->transformType = transType;
}

bool
StartTransformationGenerator3d::compute(const PointRepresentation * pointSet1, 
                                        const PointRepresentation * pointSet2,
                                        McDArray<Transformation>          & startTransformations,
                                        McDArray<int>                     & cliqueSizes)
{
    // initialize connectivity of correspondence graph
    //   - each node is connected to itself; 
    //     this is needed for the clique detection algorithm
    McDArray<McBitfield> connected;

    McDArray<McVec2i> corrGraph;
    pointSet1->computeCorrespondenceGraphVertices(pointSet2,
                                                  corrGraph);

    // compute edges of correspondence graph
    pointSet1->computeCorrespondenceGraphEdges(pointSet2, 
                                               corrGraph,
                                               connected);
    
    McDArray<int> notAndCand(corrGraph.size());
    for ( int i=0; i<corrGraph.size(); i++ ) {
        notAndCand[i] = i;
    }

    int nCliques = 0;
    McDArray<int> clique;
    McDArray<McDArray<int> > cliques;
    bool returnStatus = 
        McCliqueDetection::computeCliquesBronKerbosch(clique, 
                                                      notAndCand, 
                                                      0, 
                                                      notAndCand.size(),
                                                      connected, 
                                                      nCliques, 
                                                      cliques, 
                                                      minCliqueSize,
                                                      maxNumStartTransformations);

    const McDArray<McVec3f> & coordsOfReference = pointSet1->getCoords();
    const McDArray<McVec3f> & coordsOfQuery     = pointSet2->getCoords();

    McDArray<int> pointsOfReference;
    McDArray<int> pointsOfQuery;
    McMat4f transform;
    Transformation transformation;

    // compute start transformations
    startTransformations.remax(cliques.size());
    for ( int i=0; i<cliques.size(); i++ ) {
        // check whether point sets found using clique detection are
        // really close enough
        pointsOfReference.resize(cliques[i].size());
        pointsOfQuery.resize(cliques[i].size());
        for ( int j=0; j<cliques[i].size(); j++ ) {
            pointsOfReference[j] = corrGraph[cliques[i][j]][0];
            pointsOfQuery[j]     = corrGraph[cliques[i][j]][1];
        }

        mcAlignPointSets(transform, 
                         &coordsOfReference[0], 
                         &coordsOfQuery[0], 
                         pointsOfReference, 
                         pointsOfQuery, 
                         pointsOfReference.size(), 
                         transformType);

        transformation.setTransformation3d(transform);
        startTransformations.append(transformation);
        cliqueSizes.append(cliques[i].size());
    }

    return returnStatus;
}
  
bool StartTransformationGenerator3d::compute(const PointRepresentation * pointSet1, 
                                             const PointRepresentation * pointSet2,
                                             McDArray<Transformation>          & startTransformations)
{
    McDArray<int> cliqueSizes;
    return compute(pointSet1, pointSet2, startTransformations, cliqueSizes);
}
