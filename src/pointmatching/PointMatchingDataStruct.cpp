/////////////////////////////////////////////////////////////////
// 
// PointMatchingDataStruct.cpp
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include <mclib/McMat4.h>
#include <mclib/internal/McAlignPointSets.h>

#include "PointMatchingDataStruct.h"

PointMatchingDataStruct::PointMatchingDataStruct()
{
    score = -1.0f;
}

void 
PointMatchingDataStruct::reset()
{
    score = -1.0f;
    refPoints.clear();
    queryPoints.clear();
}

PointMatchingDataStruct * 
PointMatchingDataStruct::duplicate() const
{
    PointMatchingDataStruct * pointMatching =
        new PointMatchingDataStruct();
    
    *pointMatching = *this;

    return pointMatching;
}

PointMatchingDataStruct & 
PointMatchingDataStruct::operator=(const PointMatchingDataStruct & pointMatching)
{
    this->score       = pointMatching.getScore();
    this->refPoints   = pointMatching.getRefPoints();
    this->queryPoints = pointMatching.getQueryPoints();
    
    return *this;
}
    
void 
PointMatchingDataStruct::setScore(const double score)
{
    this->score = score;
}

double 
PointMatchingDataStruct::getScore() const
{
    return score;
}

int
PointMatchingDataStruct::getSize() const
{
    return refPoints.size();
}

void
PointMatchingDataStruct::setRefPoints(const McDArray<int> & refPoints)
{
    // copy points
    this->refPoints = refPoints;
}

void
PointMatchingDataStruct::setQueryPoints(const McDArray<int> & queryPoints)
{
    // copy points
    this->queryPoints = queryPoints;
}

const McDArray<int> &
PointMatchingDataStruct::getRefPoints() const
{
    return refPoints;
}

const McDArray<int> &
PointMatchingDataStruct::getQueryPoints() const
{
    return queryPoints;
}

