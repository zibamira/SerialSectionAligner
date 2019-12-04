/////////////////////////////////////////////////////////////////
// 
// AngularCacheObject3d.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef ANGULAR_CACHE_OBJECT_3D_H
#define ANGULAR_CACHE_OBJECT_3D_H

#include <mclib/McVec2i.h>
#include <mclib/McVec3.h>
#include <mclib/McMat4.h>
#include <mclib/McDArray.h>
#include <mclib/McBitfield.h>

#include "api.h"
#include "CacheObject.h"

class POINTMATCHING_API AngularCacheObject3d : public CacheObject 
{
public:
    AngularCacheObject3d();
    ~AngularCacheObject3d();

    void setCoordsAndDirections(
            const McDArray<McVec3f> & coordinates,
            const McDArray<McVec3f> & origCoordinates,
            const McDArray<McVec3f> & directions,
            const McMat4f           & transform);

    const McDArray<McVec3f> & getCoords() const;

    const McDArray<McVec3f> & getDirections() const;

    void computeDistancesAndSort(const McDArray<McVec3f> & coords,
                                 const McDArray<McVec3f> & origCoords,
                                 const McDArray<McVec3f> & directions1,
                                 const double              maxDistance,
                                 const double              minAngleForOptMatching);

    bool getNextMatchingPair(int    & set1PointIx,
                             int    & set2PointIx,
                             double & dist2);

protected:
    McDArray<McVec3f> mCoords;
    McDArray<McVec3f> mOrigCoords;
    McDArray<McVec3f> mDirections;
    
    double getDistance(const McVec3f coord1, const McVec3f coord2, const McVec3f origCoord1, const McVec3f origCoord2, const McVec3f dir1, const McVec3f dir2, double maxDistance);
  
private:
    McDArray<McVec2i> pointCorrespondence;
    McDArray<double>  pointDist2;
    McDArray<int>     sortedDist2List;
    McBitfield        pointSet1IsMatched;
    McBitfield        pointSet2IsMatched;
    int               currListIndex;
};
#endif
