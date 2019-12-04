#ifndef HX_TEST_POINT_MATCHING_H
#define HX_TEST_POINT_MATCHING_H

#include <hxcore/HxCompModule.h>
#include <hxcore/HxPortDoIt.h>

class HxTest_PointMatching : public HxCompModule 
{
    HX_HEADER(HxTest_PointMatching);

public:

    /// Start computation.
    HxPortDoIt portAction;

    /// Compute method that handles all the computation.
    virtual void compute();
};

#endif
