/////////////////////////////////////////////////////////////////
// 
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include <hxcore/HxObject.h>

#include <hxtemplatematchingutil/mcutil.h>
#include "HxPortScanConvertCylinder.h"

HxPortScanConvertCylinder::HxPortScanConvertCylinder(HxObject * parent) :
    portCylinderLength                 (parent, "cylinderLength", "Cylinder Length"),
    portMaskLength                     (parent, "maskLength", "Mask Length"),
	portPhiRange					   (parent, "phiRange", "Phi Range", 2),
	portThetaRange					   (parent, "thetaRange", "Theta Range", 2),
    portAngleIncrement                 (parent, "angleIncrement", "Angle Increment")
{
    portCylinderLength.setMinMax(1.0f, 50.0f);
    portCylinderLength.setValue(10.0f);
    portMaskLength.setMinMax(1.0f, 50.0f);
    portMaskLength.setValue(10.0f);

	portPhiRange.setLabel("Range Phi");
    portPhiRange.setMinMax(0,0,180);
    portPhiRange.setMinMax(1,0,180);
	portPhiRange.setLabel(0,"min");
	portPhiRange.setValue(0,0);
	portPhiRange.setLabel(1,"max");
	portPhiRange.setValue(1,180);

	portThetaRange.setLabel("Range Theta");
    portThetaRange.setMinMax(0,-90,90);
    portThetaRange.setMinMax(1,-90,90);
    portThetaRange.setLabel(0,"min");
	portThetaRange.setValue(0,-90);
	portThetaRange.setLabel(1,"max");
	portThetaRange.setValue(1,90);

    portAngleIncrement.setLabel("Angle step size:");
    portAngleIncrement.setMinMax(1.0f, 180.0f);
    portAngleIncrement.setValue(5.0f);

    portMaskLength.setMinMax(0,100);
    portMaskLength.hide();
    
    progressInterface = 0;
}

HxPortScanConvertCylinder::~HxPortScanConvertCylinder()
{
    // empty
}

void 
HxPortScanConvertCylinder::setProgressInterface(McProgressInterface * progressInterface)
{
    this->progressInterface = progressInterface;
}

float 
HxPortScanConvertCylinder::getCylinderLength()
{
    return portCylinderLength.getValue();
}

void 
HxPortScanConvertCylinder::compute()
{
    // empty
}

void 
HxPortScanConvertCylinder::update()
{
    portCylinderLength.show();
    portAngleIncrement.show();
}

int 
HxPortScanConvertCylinder::isNew(unsigned int mask) const
{
    unsigned int touchFlags = 0;
    
    if ( portCylinderLength.isNew() )
        touchFlags |= NEW_CYLINDER_SIZE;
    
    if ( portAngleIncrement.isNew() ) 
        touchFlags |= ~NEW_CYLINDER_SIZE;
    
    return ( touchFlags & mask );
}

const float HxPortScanConvertCylinder::getAngleIncrement()
{
    return portAngleIncrement.getValue();
}

void 
HxPortScanConvertCylinder::getFieldDims(int fieldDims[3])
{
    fieldDims[0] = cylinderFieldDims[0];
    fieldDims[1] = cylinderFieldDims[1];
    fieldDims[2] = cylinderFieldDims[2];
}

const McDArray<McDArray<float> > & 
HxPortScanConvertCylinder::getNormalizedCylinderFields()
{
    return normalizedCylinderFields;
}

const McDArray<McDArray<float> > & 
HxPortScanConvertCylinder::getCylinderFields()
{
    return cylinderFields;   
}

const McDArray<McDArray<int> > & 
HxPortScanConvertCylinder::getCylinderMaskElements()
{
    return cylinderMaskElements;
}

const McDArray<McVec2f> & 
HxPortScanConvertCylinder::getCylinderOrientations()
{
    return cylinderOrientations;
}

void 
HxPortScanConvertCylinder::normalizeScanConvertedFields()
{
    /* normalize all cylinder fields to a mean value of 0.0 and a
       standard deviation of 1.0 */
    if ( progressInterface )
        progressInterface->startWorkingNoStop("Normalizing scan converted cylinder fields...");
    
    int nFields = cylinderFields.size();

    normalizedCylinderFields.resize(nFields);
    
    int i;
    for ( i=0; i<nFields; i++ ) {
        
        if ( progressInterface )
            progressInterface->setProgressValue(float(i)/float(nFields));
        
        normalizedCylinderFields[i] = cylinderFields[i];

		McDArray<float> selectedNormCylField = mcutil::GetSelection(normalizedCylinderFields[i],cylinderMasks[i]);
		mcutil::normalizeValues(&selectedNormCylField);
		mcutil::InsertSelection(selectedNormCylField,cylinderMasks[i],normalizedCylinderFields[i]);
    }
    
    if ( progressInterface )
        progressInterface->stopWorking();
}


void 
HxPortScanConvertCylinder::getPlaneDims(int fieldDims[3])
{
    fieldDims[0] = planeDims[0];
    fieldDims[1] = planeDims[1];
    fieldDims[2] = planeDims[2];
}

const McDArray< McDArray<McDArray<float> > > & 
HxPortScanConvertCylinder::getNormalizedPlaneFields()
{
    return normalizedPlaneFields;
}

const McDArray< McDArray<McDArray<float> > > & 
HxPortScanConvertCylinder::getPlaneFields()
{
    return planeFields;   
}

const McDArray< McDArray<McDArray<int> > > & 
HxPortScanConvertCylinder::getPlaneMaskElements()
{
    return planeMaskElements;
}

void 
HxPortScanConvertCylinder::normalizeScanConvertedPlane()
{
    /* normalize all cylinder fields to a mean value of 0.0 and a
       standard deviation of 1.0 */
    int nFields = planeFields.size();

    normalizedPlaneFields.resize(nFields);
    
    for (int i=0; i<nFields; i++ ) 
    {
        int nPlanes = planeFields[i].size();
        normalizedPlaneFields[i].resize(nPlanes);
        for (int j=0; j<nPlanes; j++ )
        {
            normalizedPlaneFields[i][j].resize(0);
            normalizedPlaneFields[i][j].appendArray(planeFields[i][j]);

            const McBitfield& mask = planeMasks[i][j];
            const McDArray<float>& data = normalizedPlaneFields[i][j];
		    McDArray<float> selectedNormField = mcutil::GetSelection(data,mask);
		    mcutil::normalizeValues(&selectedNormField);
		    mcutil::InsertSelection(selectedNormField,planeMasks[i][j],normalizedPlaneFields[i][j]);
        }
    }
}

void 
HxPortScanConvertCylinder::computeCylinderFieldDims(const float cylLength,
                                                    const float cylRadius,
                                                    int         dims[3])
{
    // compute length of bounding box of cylinder field
    float cylLength2 = cylLength/2.0f;
    float bboxLength = 2.0f * sqrt(   (cylLength2 * cylLength2) 
                                    + (cylRadius  * cylRadius) );
    
    // compute dims of cylinder field
    int i;
    for ( i=0; i<3; i++ ) {
        dims[i] = int(bboxLength) + 1;
        if ( ( dims[i] / 2 ) * 2 == dims[i] ) {
            dims[i]++;    // dims[i] should be odd
        } else {
            dims[i] += 2; // dims[i] is already odd, but should be enlarged slightly
        }
    }
}

McMat4f
HxPortScanConvertCylinder::computeTransformMatrix(const float     phi,
                                                  const float     theta,
                                                  const float     psi,
                                                  const McVec3f & cylinderCenter)
{
    float a[3][3];
    
    float phiRad   = (phi   / 180.0) * M_PI;
    float thetaRad = (theta / 180.0) * M_PI;
    // varying psi for a cylinder is not needed, since the cylinder is
    // rotationally symmetric -> so we just set it to a fixed value,
    // in this case 0
    const float psiRad   = (psi / 180.0) * M_PI;
    
    a[0][0] = cos(thetaRad)*cos(phiRad);
    a[0][1] = cos(thetaRad)*sin(phiRad);
    a[0][2] = -sin(thetaRad);
    a[1][0] = sin(psiRad)*sin(thetaRad)*cos(phiRad) - cos(psiRad)*sin(phiRad);
    a[1][1] = sin(psiRad)*sin(thetaRad)*sin(phiRad) + cos(psiRad)*cos(phiRad);
    a[1][2] = cos(thetaRad)*sin(psiRad);
    a[2][0] = cos(psiRad)*sin(thetaRad)*cos(phiRad) + sin(psiRad)*sin(phiRad);
    a[2][1] = cos(psiRad)*sin(thetaRad)*sin(phiRad) - sin(psiRad)*cos(phiRad);
    a[2][2] = cos(thetaRad)*cos(psiRad);

    McMat4f mat;
    mat = McMat4f(a[0][0],a[0][1],a[0][2],0,
                  a[1][0],a[1][1],a[1][2],0,
                  a[2][0],a[2][1],a[2][2],0,
                  cylinderCenter[0], cylinderCenter[1], cylinderCenter[2], 1);
    
    return mat;
}

void HxPortScanConvertCylinder::clearMemberVariables()
{
    cylinderOrientations.clear();
    cylinderMasks.clear();
    cylinderMaskElements.clear();
    cylinderFields.clear();
    normalizedCylinderFields.clear();
    planeFields.clear();
    planeMaskElements.clear();
    planeMasks.clear();
    normalizedPlaneFields.clear();
}

