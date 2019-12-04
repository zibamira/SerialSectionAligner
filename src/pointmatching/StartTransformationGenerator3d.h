/////////////////////////////////////////////////////////////////
// 
// StartTransformationGenerator3d.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef START_TRANSFORMATION_GENERATOR_3D_H
#define START_TRANSFORMATION_GENERATOR_3D_H

#include <mclib/McHandable.h>

#include "SimplePointRepresentation3d.h"
#include "Transformation.h"
#include "api.h"

class POINTMATCHING_API StartTransformationGenerator3d : public McHandable 
{
public:
    StartTransformationGenerator3d();

    void setMinimumCliqueSize(const int size);

    void setMaxNumStartTransformations(const int numStartTransformations);
    
    /// Type of transformation used to align points.
    /// 0=rigid, 1=rigid + iso-scale, 2=affine. 
    /// Default is rigid.
    void setTransformType(const int transType);

    bool compute(const PointRepresentation * pointSet1, 
                 const PointRepresentation * pointSet2,
                 McDArray<Transformation>          & startTransformations);
    
    // Same as above, but returns statistics information:
    // size of clique corresponding to each starting transformation
    bool compute(const PointRepresentation * pointSet1, 
                 const PointRepresentation * pointSet2,
                 McDArray<Transformation>          & startTransformations,
                 McDArray<int>                     & cliqueSizes);
    
private:
    int   minCliqueSize;
    int   maxNumStartTransformations;

    int   transformType;
};

#endif
