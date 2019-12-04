/////////////////////////////////////////////////////////////////
// 
// AngularCacheObject3d.cpp
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include <mclib/internal/McSorter.h>
#include <mclib/internal/McComparators.h>
#include <mclib/McPlane.h>
#include <mclib/McLine.h>
#include <mclib/McMath.h>

#include "AngularCacheObject3d.h"

AngularCacheObject3d::AngularCacheObject3d()
{
    // empty
}

AngularCacheObject3d::~AngularCacheObject3d()
{
    // empty
}

void 
AngularCacheObject3d::setCoordsAndDirections(
            const McDArray<McVec3f> & coordinates,
            const McDArray<McVec3f> & origCoordinates,
            const McDArray<McVec3f> & directions,
            const McMat4f           & transform)
{

    assert(coordinates.size()==directions.size());
    mCoords.resize(coordinates.size());
    mOrigCoords.resize(origCoordinates.size());
    mDirections.resize(directions.size());

    // transform coordinates
    for ( int i=0; i<coordinates.size(); i++ ) {
        McVec3f dir = coordinates[i] + directions[i];
        McVec3f newDir;
        transform.multVecMatrix(dir, newDir);
        transform.multVecMatrix(coordinates[i], mCoords[i]);
        transform.multVecMatrix(origCoordinates[i], mOrigCoords[i]);
        newDir = newDir - mCoords[i];
        newDir.normalize();
        mDirections[i] = newDir;

    }

}
 
const McDArray<McVec3f> & 
AngularCacheObject3d::getCoords() const
{
    return mCoords;
}

const McDArray<McVec3f> & 
AngularCacheObject3d::getDirections() const
{
    return mDirections;
}

void 
AngularCacheObject3d::computeDistancesAndSort(
    const McDArray<McVec3f> & coords1,
    const McDArray<McVec3f> & origCoords1,
    const McDArray<McVec3f> & directions1,
    const double              maxDistance,
    const double              minAngleForOptMatching)
{
    const McDArray<McVec3f> & coords2 = this->mCoords;
    const McDArray<McVec3f> & directions2 = this->mDirections;

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
            dist2 = getDistance(coords1[i] ,coords2[j], origCoords1[i], mOrigCoords[j], directions1[i], directions2[j], maxDistance);
            if ( dist2 < maxDistance2 ) {
                double angle = directions1[i].dot(directions2[j]);
                angle = acos(angle);
                angle = angle/(2*M_PI)*360.0;
                // Britta, why must the angle be larger than minAngleForOptMatching?
                // I would have thought, that the angle should be smaller than
                // a predefined angle?  Or do you assume that the direction vectors 
                // point in opposite directions? [ Daniel ]
                if( angle > minAngleForOptMatching)
                {
                    pointCorrespondence.append(McVec2i(i,j));
                    pointDist2.append(dist2);
                }
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

double AngularCacheObject3d::getDistance(
    const McVec3f coord1,
    const McVec3f coord2,
    const McVec3f origCoord1,
    const McVec3f origCoord2,
    const McVec3f dir1,
    const McVec3f dir2,
    double        maxDistance)
{
    //project coord1 on plane perpendicular to dir2 positioned at coord2

    if ((coord1-coord2).length()>maxDistance*3)
        return FLT_MAX;

    //check, if the two lines overlapp: 
    //if  dir1 points into positive z, then origPoint2 must be below origPoint1, 
    if (dir1.z>=0)
    {
        if (origCoord1.z<origCoord2.z)
            return FLT_MAX;
    }
    else
    {
        if (origCoord1.z>origCoord2.z)
            return FLT_MAX;
    }

    McPlane plane(dir2,coord2);
    McLine line(coord1, coord1 + dir1);
    McVec3f intersectionPoint;
    if (plane.intersect(line, intersectionPoint))
    {
        return (coord2-intersectionPoint).length2();
    }
    else
    {
        return FLT_MAX;
    }

}


bool
AngularCacheObject3d::getNextMatchingPair(
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
