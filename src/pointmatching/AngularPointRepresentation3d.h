/////////////////////////////////////////////////////////////////
// 
// AngularPointRepresentation3d.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef ANGULAR_POINT_REPRESENTATION_3D_H
#define ANGULAR_POINT_REPRESENTATION_3D_H

#include <mclib/McVec3.h>
#include <mclib/McVec2i.h>
#include <mclib/McBox3f.h>
#include <mclib/McDArray.h>
#include <mclib/McBitfield.h>

#include "PointRepresentation.h"
#include "api.h"

class POINTMATCHING_API AngularPointRepresentation3d : public PointRepresentation {

public:
    AngularPointRepresentation3d(const double maxPointDistance,
                                 const double cliqueDistThreshold,
                                 const double maxAngleDistForInitMatching,
                                 const double minAngleForOptMatching,
                                 const double maxDistForAngle);

    void setCoords(const McDArray<McVec3f> & coords);
    void setOrigCoords(const McDArray<McVec3f> & origCoords);
    void setDirections(const McDArray<McVec3f> & directions);
    const McDArray<McVec3f> & getCoords() const;
    const McDArray<McVec3f> & getOrigCoords() const;
    const McDArray<McVec3f> & getDirections() const;
    const McDArray<McDArray<double> > & getDistMatrix() const;

    virtual int getNumPoints() const;

    virtual void computeCorrespondenceGraphVertices(
                const PointRepresentation * pointSet2, 
                McDArray<McVec2i>                 & corrGraph) const;

    virtual void computeCorrespondenceGraphEdges(
                const PointRepresentation * pointSet2, 
                const McDArray<McVec2i>           & corrGraph,
                McDArray<McBitfield>              & connected) const;

    virtual void computeTransformation(
                const PointRepresentation     * pointSet2,
                const PointMatchingDataStruct * pointMatching,
                Transformation                * pointSet2Transform) const;

    /* Virtual functions inherited from PointRepresentation. */

    virtual void setScoreDivisor(
                const PointRepresentation    * pointSet2, 
                PointMatchingScoringFunction * scoringFunction) const;

    virtual CacheObject * startGreedyPointMatching(
                const PointRepresentation * pointSet2,
                const Transformation      * pointSet2StartTransform) const;

    virtual CacheObject * startExactPointMatching(
                const PointRepresentation * pointSet2,
                const Transformation      * pointSet2StartTransform) const;

    virtual bool getNextMatchingPair(
                CacheObject * cacheObject,
                int         & refPointIx,
                int         & queryPointIx,
                double      & dist2) const;

    virtual void finishGreedyPointMatching() const;

    virtual void finishExactPointMatching() const;

    bool canPointsBeMatched(const int pointRep1PointIdx,
			    const PointRepresentation * pointRep2,
			    const int pointRep2PointIdx,
			    CacheObject * cacheObject,
			    double & squaredEdgeWeight) const;

    double getSquaredEdgeWeight(const int pointRep1PointIdx,
			       const PointRepresentation * pointRep2,
			       const int pointRep2PointIdx,
			       CacheObject * cacheObject) const;

    /// Type of transformation used to align points.
    /// 0=rigid, 1=rigid + iso-scale, 2=affine. 
    /// Default is rigid.
    void setTransformType(const int transType);

    bool connectByEdge( const int vertex1, 
                        const int vertex2, 
                        const PointRepresentation * otherPoints,
                        const McDArray<McVec2i>   & corrGraph) const;

protected:
    /// Compute bounding box of coordinates and distance matrix.
    void computeBBoxAndDistMatrix();

private:
    McDArray<McVec3f>           mCoords;
    McDArray<McVec3f>           mOrigCoords;
    McDArray<McVec3f>           mDirections;
    McBox3f                     bbox;
    double                      maximumDistance;
    McDArray<McDArray<double> > distMatrix;
    int                         transformType;
    double                      mMaxPointDistance;
    double                      mMaxPointDistance2;
    double                      mCliqueDistThreshold;
    double                      mMaxAngleDistForInitMatching;
    double                      mMinAngleForOptMatching;
    double                      mMaxDistForAngle;
};

#endif
