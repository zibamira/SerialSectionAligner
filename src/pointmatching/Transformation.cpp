/////////////////////////////////////////////////////////////////
// 
// Transformation.cpp
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include "Transformation.h"

Transformation::Transformation()
{
    // empty
}

void 
Transformation::setTransformation2d(McMat3f & matrix)
{
    this->matrix2d = matrix;
}
    
const McMat3f &
Transformation::getTransformation2d() const
{
    return matrix2d;
}

void 
Transformation::setTransformation3d(McMat4f & matrix)
{
    this->matrix3d = matrix;
}

const McMat4f &
Transformation::getTransformation3d() const
{
    return matrix3d;
}
