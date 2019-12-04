/////////////////////////////////////////////////////////////////
// 
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef HX_PORT_SCAN_CONVERT_HOLLOW_CYLINDER_H
#define HX_PORT_SCAN_CONVERT_HOLLOW_CYLINDER_H

#include <hxcore/HxPortFloatSlider.h>

#include <hxtemplatematchingutil/HxPortScanConvertCylinder.h>
#include "api.h"

class HxObject;

class HXTEMPLATEMATCHINGUTIL_API HxPortScanConvertHollowCylinder : public HxPortScanConvertCylinder
{
public:
    /// Constructor.
    HxPortScanConvertHollowCylinder(HxObject * parent);

    /// Update method that handles all the GUI and further updates.
    virtual void update();

    /// Compute method that handles all the computation.
    virtual void compute();

    /// Check whether any port is new.
    virtual int isNew(unsigned int mask=0xffffffff) const;

    /// Virtual function to set the gray value range. These values
    /// might be passed on to port etc.
    virtual void setGrayValueRange(const float min,
                                   const float max);
    
    /// Virtual function to call scan conversion of cylinders.
    virtual void computeScanConversions();
    
    /// Return cylinder radius.
    virtual float getCylinderRadius();
    
    /// Return mask radius.
    virtual float getMaskRadius(){return portMaskCylinderRadius.getValue();};
    
public:
    /// Relative radius of cylinder mask: this value is multiplied by
    /// the voxel size to yield the actual radius.
    HxPortFloatSlider portMaskCylinderRadius;

    /// Relative radius of outer cylinder: this value is multiplied by
    /// the voxel size to yield the actual radius.
    HxPortFloatSlider portOuterCylinderRadius;

    /// Relative radius of inner cylinder: this value is multiplied by
    /// the voxel size to yield the actual radius.
    HxPortFloatSlider portInnerCylinderRadius;

    /// Background, i.e. gray value of everything that is not cylinder..
    HxPortFloatSlider portBackgroundGrayValue;  
    
    /// Foreground, i.e. cylinder gray value.
    HxPortFloatSlider portOuterCylinderGrayValue;

    /// Foreground, i.e. cylinder gray value.
    HxPortFloatSlider portInnerCylinderGrayValue;

protected:
    /// Destructor.
    virtual ~HxPortScanConvertHollowCylinder();  
};

#endif
