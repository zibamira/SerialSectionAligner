/////////////////////////////////////////////////////////////////
// 
// Transformation.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <mclib/McHandable.h>
#include <mclib/McMat3f.h>
#include <mclib/McMat4.h>

#include "api.h"

class POINTMATCHING_API Transformation : McHandable
{
public:
    Transformation();

    void setTransformation2d(McMat3f & matrix);
    const McMat3f & getTransformation2d() const;

    void setTransformation3d(McMat4f & matrix);
    const McMat4f & getTransformation3d() const;

private:
    McMat3f matrix2d;
    McMat4f matrix3d;
};

#endif
