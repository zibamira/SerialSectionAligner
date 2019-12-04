/////////////////////////////////////////////////////////////////
// 
// SimpleCacheObject3d.cpp
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include <mclib/internal/McSorter.h>
#include <mclib/internal/McComparators.h>

#include "SimpleCacheObject3d.h"

SimpleCacheObject3d::SimpleCacheObject3d()
{
    // empty
}

SimpleCacheObject3d::~SimpleCacheObject3d()
{
    // empty
}

void 
SimpleCacheObject3d::setCoords(
            const McDArray<McVec3f> & coordinates,
            const McMat4f           & transform)
{
    coords.resize(coordinates.size());

    // transform coordinates
    for ( int i=0; i<coords.size(); i++ ) {
        transform.multVecMatrix(coordinates[i], coords[i]);
    }
}
 
const McDArray<McVec3f> & 
SimpleCacheObject3d::getCoords() const
{
    return coords;
}

void 
SimpleCacheObject3d::computeDistancesAndSort(
            const McDArray<McVec3f> & coords1,
            const double              maxDistance)
{
    const McDArray<McVec3f> & coords2 = this->coords;

    const int numCoords1 = coords1.size();
    const int numCoords2 = coords2.size();

    pointSet1IsMatched.resize(numCoords1);
    pointSet1IsMatched.unsetAll();
    pointSet2IsMatched.resize(numCoords2);
    pointSet2IsMatched.unsetAll();
    currListIndex = 0;

    pointCorrespondence.clear();
    pointDist2.clear();

    double dist2;    
    double maxDistance2 = maxDistance * maxDistance;

    for ( int i=0; i<numCoords1; ++i ) {
        for ( int j=0; j<numCoords2; ++j ) {
            dist2 = (coords1[i] - coords2[j]).length2();
            if ( dist2 < maxDistance2 ) {
                pointCorrespondence.append(McVec2i(i,j));
                pointDist2.append(dist2);
            }
        }
    }

    sortedDist2List.resize(pointCorrespondence.size());
    for ( int i=0; i<sortedDist2List.size(); ++i ) {
        sortedDist2List[i] = i;   
    }

    if (pointDist2.size() > 0) {
        McIndexByValueComparator<double> comparator(&pointDist2[0]);
        sort(&(sortedDist2List[0]), sortedDist2List.size(), comparator);
    }
}

bool
SimpleCacheObject3d::getNextMatchingPair(
            int    & set1PointIx,
            int    & set2PointIx,
            double & dist2)
{
    if ( currListIndex < sortedDist2List.size() ) {
        int ix1, ix2;
        do {
            ix1 = pointCorrespondence[sortedDist2List[currListIndex]][0];
            if ( pointSet1IsMatched[ix1] ) {
                currListIndex++;
            } else {
                ix2 = pointCorrespondence[sortedDist2List[currListIndex]][1];
                if ( pointSet2IsMatched[ix2] ) {
                    currListIndex++;
                } else {
                    set1PointIx = ix1;
                    set2PointIx = ix2;
                    dist2 = pointDist2[sortedDist2List[currListIndex]];
                    pointSet1IsMatched.set(ix1);
                    pointSet2IsMatched.set(ix2);
                    return true;
                }
            }
        } while ( currListIndex < sortedDist2List.size() );
    }

    return false;
}
