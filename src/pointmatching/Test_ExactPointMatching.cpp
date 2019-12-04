#include <mclib/McDArray.h>
#include <mclib/McVec3.h>
#include <mclib/McMat4.h>

#include "SimplePointRepresentation3d.h"
#include "ExactPointMatchingAlgorithm.h"
#include "Transformation.h"
#include "PointMatchingDataStruct.h"
#include "Test_ExactPointMatching.h"

namespace
{

void getCoords1(McDArray<McVec3f> & coords)
{
    coords.append(McVec3f(1.0, 1.0, 0.0));
    coords.append(McVec3f(2.0, 1.0, 0.0));
    coords.append(McVec3f(3.0, 1.0, 0.0));
    coords.append(McVec3f(4.0, 1.0, 0.0));
}

void getCoords2(McDArray<McVec3f> & coords)
{
    coords.append(McVec3f(1.0, 2.0, 0.0));
    coords.append(McVec3f(2.0, 2.0, 0.0));
    coords.append(McVec3f(3.0, 2.0, 0.0));
    coords.append(McVec3f(4.0, 2.0, 0.0));
}

void getCoords3(McDArray<McVec3f> & coords)
{
    coords.append(McVec3f(2.0, 1.0, 0.0));
    coords.append(McVec3f(3.0, 1.0, 0.0));
    coords.append(McVec3f(4.0, 1.0, 0.0));
    coords.append(McVec3f(5.0, 1.0, 0.0));
}

void getCoords4(McDArray<McVec3f> & coords)
{
    coords.append(McVec3f(2.0, 2.0, 0.0));
    coords.append(McVec3f(3.0, 2.0, 0.0));
    coords.append(McVec3f(4.0, 2.0, 0.0));
    coords.append(McVec3f(5.0, 2.0, 0.0));
}

void getCoords5(McDArray<McVec3f> & coords)
{
    coords.append(McVec3f(3.0, 2.0, 0.0));
    coords.append(McVec3f(4.0, 2.0, 0.0));
    coords.append(McVec3f(5.0, 2.0, 0.0));
    coords.append(McVec3f(6.0, 2.0, 0.0));
}

void getCoords6(McDArray<McVec3f> & coords)
{
    coords.append(McVec3f(1.55, 2.0, 0.0));
    coords.append(McVec3f(2.55, 2.0, 0.0));
    coords.append(McVec3f(3.55, 2.0, 0.0));
    coords.append(McVec3f(4.55, 2.0, 0.0));
}

void getIdentityTransformation(Transformation & transform)
{
    McMat4f transf = McMat4f::IDENTITY;
    transform.setTransformation3d(transf);
}

void getUserDefinedEdgeWeights(const McDArray<McVec3f> & coords1,
                               const McDArray<McVec3f> & coords2,
                               McDArray< McDArray<double> > & edgeWeights)
{
    edgeWeights.resize(coords2.size());
    for ( int i=0; i<coords2.size(); ++i )
    {
        edgeWeights[i].resize(coords1.size());
        edgeWeights[i].fill(2.f);
        if ( i < coords1.size() )
            edgeWeights[i][i] = 1.f;
    }
}

}

bool Test_ExactPointMatching::test1()
{
    printf("Test_ExactPointMatching::test1\n");

    McDArray<McVec3f> coords1;
    getCoords1(coords1);

    McDArray<McVec3f> coords2;
    getCoords2(coords2);

    return ( test(coords1, coords2) &&
             testMaxCardinality(coords1, coords2) &&
             testUserDefinedEdgeWeights(coords1, coords2) );
}

bool Test_ExactPointMatching::test2()
{
    printf("Test_ExactPointMatching::test2\n");

    McDArray<McVec3f> coords1;
    getCoords1(coords1);

    McDArray<McVec3f> coords2;
    getCoords3(coords2);

    return ( test(coords1, coords2) &&
             testMaxCardinality(coords1, coords2) &&
             testUserDefinedEdgeWeights(coords1, coords2) );
}

bool Test_ExactPointMatching::test3()
{
    printf("Test_ExactPointMatching::test3\n");

    McDArray<McVec3f> coords1;
    getCoords1(coords1);

    McDArray<McVec3f> coords2;
    getCoords4(coords2);

    return ( test(coords1, coords2) &&
             testMaxCardinality(coords1, coords2) &&
             testUserDefinedEdgeWeights(coords1, coords2) );
}

bool Test_ExactPointMatching::test4()
{
    printf("Test_ExactPointMatching::test4\n");

    McDArray<McVec3f> coords1;
    getCoords1(coords1);

    McDArray<McVec3f> coords2;
    getCoords5(coords2);

    return ( test(coords1, coords2) &&
             testMaxCardinality(coords1, coords2) &&
             testUserDefinedEdgeWeights(coords1, coords2) );
}

bool Test_ExactPointMatching::test5()
{
    printf("Test_ExactPointMatching::test4\n");

    McDArray<McVec3f> coords1;
    getCoords1(coords1);

    McDArray<McVec3f> coords2;
    getCoords6(coords2);

    return ( test(coords1, coords2) &&
             testMaxCardinality(coords1, coords2) &&
             testUserDefinedEdgeWeights(coords1, coords2) );
}

bool Test_ExactPointMatching::test(const McDArray< McVec3f > & coords1,
				   const McDArray< McVec3f > & coords2)
{
    SimplePointRepresentation3d pointRep1(4.0, 0.0);
    pointRep1.setCoords(coords1);

    SimplePointRepresentation3d pointRep2(4.0, 0.0);
    pointRep2.setCoords(coords2);
    
    Transformation pointRep2Transform;
    getIdentityTransformation(pointRep2Transform);

    PointMatchingDataStruct pointMatching;
    ExactPointMatchingAlgorithm exactAlgorithm;
    exactAlgorithm.computePointMatching(&pointRep1, 
		         		&pointRep2,
					&pointRep2Transform,
					&pointMatching);

    const McDArray<int> & refPoints = pointMatching.getRefPoints();
    const McDArray<int> & queryPoints = pointMatching.getQueryPoints();

    printf("exact (max. score):                ");
    for ( int i=0; i<refPoints.size(); ++i )
    {
        printf(" (%d, %d)", refPoints[i], queryPoints[i]);
    }
    printf("\n");
    printf("score: %f\n", pointMatching.getScore());
    fflush(stdout);

    return true;
}

bool Test_ExactPointMatching::testMaxCardinality(const McDArray< McVec3f > & coords1,
                                                 const McDArray< McVec3f > & coords2)
{
    SimplePointRepresentation3d pointRep1(4.0, 0.0);
    pointRep1.setCoords(coords1);

    SimplePointRepresentation3d pointRep2(4.0, 0.0);
    pointRep2.setCoords(coords2);
    
    Transformation pointRep2Transform;
    getIdentityTransformation(pointRep2Transform);

    PointMatchingDataStruct pointMatching;
    ExactPointMatchingAlgorithm exactAlgorithm;
    exactAlgorithm.computeMaxCardinalityPointMatching(&pointRep1, 
                                                      &pointRep2,
                                                      &pointRep2Transform,
                                                      &pointMatching);

    const McDArray<int> & refPoints = pointMatching.getRefPoints();
    const McDArray<int> & queryPoints = pointMatching.getQueryPoints();

    printf("exact (max. cardinality)          :");
    for ( int i=0; i<refPoints.size(); ++i )
    {
        printf(" (%d, %d)", refPoints[i], queryPoints[i]);
    }
    printf("\n");
    printf("score: %f\n", pointMatching.getScore());
    fflush(stdout);

    return true;
}

bool Test_ExactPointMatching::testUserDefinedEdgeWeights(const McDArray< McVec3f > & coords1,
                                                         const McDArray< McVec3f > & coords2)
{
    SimplePointRepresentation3d pointRep1(4.0, 0.0);
    pointRep1.setCoords(coords1);

    SimplePointRepresentation3d pointRep2(4.0, 0.0);
    pointRep2.setCoords(coords2);
    
    Transformation pointRep2Transform;
    getIdentityTransformation(pointRep2Transform);

    McDArray< McDArray<double> > edgeWeights;
    getUserDefinedEdgeWeights(coords1, coords2, edgeWeights);

    PointMatchingDataStruct pointMatching;
    ExactPointMatchingAlgorithm exactAlgorithm;
    exactAlgorithm.setUserDefinedEdgeWeights(edgeWeights);
    exactAlgorithm.computePointMatching(&pointRep1, 
                                        &pointRep2,
                                        &pointRep2Transform,
                                        &pointMatching);

    const McDArray<int> & refPoints = pointMatching.getRefPoints();
    const McDArray<int> & queryPoints = pointMatching.getQueryPoints();

    printf("exact (user-defined edge weights ):");
    for ( int i=0; i<refPoints.size(); ++i )
    {
        printf(" (%d, %d)", refPoints[i], queryPoints[i]);
    }
    printf("\n");
    printf("score: %f\n", pointMatching.getScore());
    fflush(stdout);

    return true;
}
