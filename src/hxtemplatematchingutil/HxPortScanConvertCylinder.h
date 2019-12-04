/////////////////////////////////////////////////////////////////
// 
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef HX_PORT_SCAN_CONVERT_CYLINDER_H
#define HX_PORT_SCAN_CONVERT_CYLINDER_H

#include <mclib/McHandable.h>
#include <mclib/McVec2.h>
#include <mclib/McMat4.h>
#include <mclib/McBitfield.h>
#include <mclib/McProgressInterface.h>

#include <hxcore/HxPortFloatSlider.h>
#include <hxcore/HxPortFloatSlider.h>
#include <hxcore/HxPortIntTextN.h>

#include "api.h"

class HxObject;

class HXTEMPLATEMATCHINGUTIL_API HxPortScanConvertCylinder : public McHandable
{
public:
    /// Constructor.
    HxPortScanConvertCylinder(HxObject * parent);

    /// Relative length of cylinder: this value is multiplied by the
    /// voxel dimension to yield the actual length.
    HxPortFloatSlider portCylinderLength;

    //mask could be shorter that template
    HxPortFloatSlider portMaskLength;

	/// Phi range.
    HxPortIntTextN portPhiRange;

	/// Theta range.
    HxPortIntTextN portThetaRange;

    /// Angle increment.
    HxPortFloatSlider portAngleIncrement;

    /// Compute method that handles all the computation.
    virtual void compute();
    
    /// Update method that handles all the GUI and further updates.
    virtual void update();

    /// Check whether any port is new.
    virtual int isNew(unsigned int mask=0xffffffff) const;
    
    /// Virtual function to set the gray value range. These values
    /// might be passed on to port etc.
    virtual void setGrayValueRange(const float min,
                                   const float max) = 0;

    /// Virtual function to call scan conversion of cylinders.
    virtual void computeScanConversions() = 0;

    /// Virtual function to call normalization of scan converted fields.
    virtual void normalizeScanConvertedFields();

    /// Compute transformation matrix that rotates around the @c
    /// cylinderCenter by two Euler angles @c phi and @c theta. Note:
    /// rotation around the third Euler angle is not necessary,
    /// because the cylinder is rotationally symmetric.
    static McMat4f computeTransformMatrix(const float     phi,
                                   const float     theta,
                                   const float     psi,
                                   const McVec3f & cylinderCenter);

    /// Clear all mmeber variables
    void clearMemberVariables();
    
    /// Return field dims.
    void getFieldDims(int fieldDims[3]);

    /// Return angle increment
    const float getAngleIncrement();
    
    /// Return normalized cylinder fields.
    const McDArray<McDArray<float> > & getNormalizedCylinderFields();

    /// Return cylinder fields.
    const McDArray<McDArray<float> > & getCylinderFields();

    /// Return cylinder mask elements.
    const McDArray<McDArray<int> > & getCylinderMaskElements();

    /// Return cylinder orientation consisting of two Euler angles.
    const McDArray<McVec2f> & getCylinderOrientations();

    /// Return plane dims.
    void getPlaneDims(int Dims[3]);
    
    /// Return normalized plane fields.
    const McDArray< McDArray<McDArray<float> > > & getNormalizedPlaneFields();

    /// Return plane fields.
    const McDArray< McDArray<McDArray<float> > > & getPlaneFields();

    /// Return plane mask elements.
    const McDArray< McDArray<McDArray<int> > > & getPlaneMaskElements();

    void normalizeScanConvertedPlane();

    /// Set progress interface.
    void setProgressInterface(McProgressInterface * progressInterface);

    /// Return cylinder length.
    virtual float getCylinderLength();

    /// Return cylinder radius.
    virtual float getCylinderRadius() = 0;

public:
    enum TouchFlags {
        NEW_CYLINDER_SIZE = 0x01
    };

protected:
    /// Destructor.
    virtual ~HxPortScanConvertCylinder();

    /// Compute required dims of cylinder field such that a cylinder
    /// with length @c cylLength and radius @c cylRadius will fit into
    /// this field no matter how it is oriented, rotated around the
    /// center. 
    void computeCylinderFieldDims(const float cylLength,
                                  const float cylRadius,
                                  int         dims[3]);

protected:
    McProgressInterface              * progressInterface;
                                       
    int                                cylinderFieldDims[3];
    int                                planeDims[3];

    McDArray<McVec2f>                  cylinderOrientations;
    McDArray<McBitfield>               cylinderMasks;
    McDArray<McDArray<int> >           cylinderMaskElements;
    McDArray<McDArray<float> >         cylinderFields;
    McDArray<McDArray<float> >         normalizedCylinderFields;
    McDArray< McDArray< McDArray<float> > > planeFields;
    McDArray< McDArray< McDArray<int> > >   planeMaskElements;
    McDArray< McDArray< McBitfield > >      planeMasks;
    McDArray< McDArray< McDArray<float> > > normalizedPlaneFields;
};

#endif
