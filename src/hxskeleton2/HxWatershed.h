#ifndef HX_WATERSHED_H
#define HX_WATERSHED_H

#include "api.h"
#include <hxcore/HxCompModule.h>
#include <hxcore/HxPortDoIt.h>
#include <hxcore/HxPortIntTextN.h>

class HXSKELETON2_API HxWatershed : public HxCompModule {

  HX_HEADER(HxWatershed);

  public:
    /// Locked cells
    HxConnection portLabels;

    /// Object to be assigned with labels (not obligatory)
    HxConnection portObject;

    /// Start the Algorithm.
    HxPortDoIt portAction;

    ///
    void compute();
};

#endif

