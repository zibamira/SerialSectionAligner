/////////////////////////////////////////////////////////////////
// 
// SimpleCacheObject3d.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef SIMPLE_CACHE_OBJECT_3D_H
#define SIMPLE_CACHE_OBJECT_3D_H

#include <mclib/McVec2i.h>
#include <mclib/McVec3.h>
#include <mclib/McMat4.h>
#include <mclib/McDArray.h>
#include <mclib/McBitfield.h>

#include "api.h"
#include "CacheObject.h"

class POINTMATCHING_API SimpleCacheObject3d : public CacheObject 
{
public:
    SimpleCacheObject3d();
    ~SimpleCacheObject3d();

    void setCoords(const McDArray<McVec3f> & coords,
                   const McMat4f           & transform);

    const McDArray<McVec3f> & getCoords() const;

    void computeDistancesAndSort(const McDArray<McVec3f> & coords,
                                 const double              maxDistance);

    bool getNextMatchingPair(int    & set1PointIx,
                             int    & set2PointIx,
                             double & dist2);

protected:
    McDArray<McVec3f> coords;
    
private:
    McDArray<McVec2i> pointCorrespondence;
    McDArray<double>  pointDist2;
    McDArray<int>     sortedDist2List;
    McBitfield        pointSet1IsMatched;
    McBitfield        pointSet2IsMatched;
    int               currListIndex;
};
#endif
