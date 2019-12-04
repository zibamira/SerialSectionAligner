/////////////////////////////////////////////////////////////////
// 
// PointMatchingDataStruct.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef POINT_MATCHING_DATA_STRUCT_H
#define POINT_MATCHING_DATA_STRUCT_H

#include <mclib/McDArray.h>
#include <mclib/McHandable.h>

#include "api.h"

class POINTMATCHING_API PointMatchingDataStruct : public McHandable {
    
public:
    PointMatchingDataStruct();

    void reset();
    PointMatchingDataStruct * duplicate() const;
    PointMatchingDataStruct & operator=(const PointMatchingDataStruct & pointMatching);

    void setRefPoints(const McDArray<int> & refPoints);
    void setQueryPoints(const McDArray<int> & queryPoints);

    const McDArray<int> & getRefPoints() const;
    const McDArray<int> & getQueryPoints() const;
    
    int getSize() const;

    void setScore(const double score);
    double getScore() const;
    
protected:
    double score;
    McDArray<int> refPoints;
    McDArray<int> queryPoints;
};

#endif
