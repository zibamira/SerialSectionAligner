/////////////////////////////////////////////////////////////////
// 
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include <mclib/McVec2.h>
#include <mclib/McVec3.h>

#include <hxcore/HxObject.h>

#include <hxtemplatematchingutil/ScanConvertCylinder.h>

#include "HxPortScanConvertHollowCylinder.h"

HxPortScanConvertHollowCylinder::HxPortScanConvertHollowCylinder(HxObject * parent) :
    HxPortScanConvertCylinder          (parent),
    portMaskCylinderRadius             (parent, "maskCylinderRadius", "Mask Cylinder Radius"),
    portOuterCylinderRadius            (parent, "outerCylinderRadius", "Outer Cylinder Radius"),
    portInnerCylinderRadius            (parent, "innerCylinderRadius", "Inner Cylinder Radius"),
    portBackgroundGrayValue            (parent, "backgroundGrayValue", "Background Gray Value"),
    portOuterCylinderGrayValue         (parent, "outerCylinderGrayValue", "Outer Cylinder Gray Value"),
    portInnerCylinderGrayValue         (parent, "innerCylinderGrayValue", "Inner Cylinder Gray Value")
{
    portMaskCylinderRadius.setMinMax(0.1f, 30.0f);
    portMaskCylinderRadius.setValue(15.0f);
    
    portOuterCylinderRadius.setMinMax(0.1f, 30.0f);
    portOuterCylinderRadius.setValue(11.0f);

    portInnerCylinderRadius.setMinMax(0.1f, 30.0f);
    portInnerCylinderRadius.setValue(8.0f);
    
    portOuterCylinderGrayValue.setLabel("Outer cylinder gray value:");
    portInnerCylinderGrayValue.setLabel("Inner cylinder gray value:");
    portBackgroundGrayValue.setLabel("Background gray value:");

    update();
}

HxPortScanConvertHollowCylinder::~HxPortScanConvertHollowCylinder()
{
    // empty
}

void 
HxPortScanConvertHollowCylinder::setGrayValueRange(const float min,
                                                  const float max)
{
    portBackgroundGrayValue.setMinMax(min, max);
    portBackgroundGrayValue.setValue((min+max)/2.0f);

    portOuterCylinderGrayValue.setMinMax(min, max);
    portOuterCylinderGrayValue.setValue((min+max)/2.0f);

    portInnerCylinderGrayValue.setMinMax(min, max);
    portInnerCylinderGrayValue.setValue((min+max)/2.0f);
}

float 
HxPortScanConvertHollowCylinder::getCylinderRadius()
{
    return portOuterCylinderRadius.getValue();
}

void 
HxPortScanConvertHollowCylinder::update()
{
    HxPortScanConvertCylinder::update();

    portMaskCylinderRadius.show();           
    portOuterCylinderRadius.show();          
    portInnerCylinderRadius.show();          
    portBackgroundGrayValue.show();          
    portOuterCylinderGrayValue.show();       
    portInnerCylinderGrayValue.show();           
}

void 
HxPortScanConvertHollowCylinder::compute()
{
    HxPortScanConvertCylinder::compute();   

    if (    portCylinderLength.isNew()
         || portMaskCylinderRadius.isNew() ) 
    {
        const float cylinderLength     = portCylinderLength.getValue();
        const float maskCylinderRadius = portMaskCylinderRadius.getValue();
        
        /* compute field dimensions */
        computeCylinderFieldDims(cylinderLength,
                                 maskCylinderRadius,
                                 cylinderFieldDims);
    }
}

int 
HxPortScanConvertHollowCylinder::isNew(unsigned int mask) const
{
    unsigned int touchFlags = 
        HxPortScanConvertCylinder::isNew(mask);
    
    if ( portMaskCylinderRadius.isNew() )
        touchFlags |= NEW_CYLINDER_SIZE;

    if ( portOuterCylinderRadius.isNew() )
        touchFlags |= NEW_CYLINDER_SIZE;

    if ( portInnerCylinderRadius.isNew() )
        touchFlags |= NEW_CYLINDER_SIZE;

    if ( portBackgroundGrayValue.isNew() ) 
        touchFlags |= ~NEW_CYLINDER_SIZE;

    if ( portOuterCylinderGrayValue.isNew() ) 
        touchFlags |= ~NEW_CYLINDER_SIZE;

    if ( portInnerCylinderGrayValue.isNew() ) 
        touchFlags |= ~NEW_CYLINDER_SIZE;

    return ( touchFlags & mask );
}

void 
HxPortScanConvertHollowCylinder::computeScanConversions()
{
    if ( progressInterface )
        progressInterface->startWorkingNoStop("Computing cylinder scan conversions..."); 

    const float cylinderLength      = portCylinderLength.getValue();
    const float maskCylinderRadius  = portMaskCylinderRadius.getValue();
    const float outerCylinderRadius = portOuterCylinderRadius.getValue();
    const float innerCylinderRadius = portInnerCylinderRadius.getValue();
    
    /* compute field dimensions */
    computeCylinderFieldDims(cylinderLength,
                             maskCylinderRadius,
                             cylinderFieldDims);
    
    /* compute field bbox and cylinder center from field dimensions */
    float   cylinderFieldBBox[6];
    McVec3f cylinderCenter;

    int i;
    for ( i=0; i<3; i++ ) {
        cylinderFieldBBox[2*i]   = 0.0f;
        cylinderFieldBBox[2*i+1] = float(cylinderFieldDims[i]-1);
        
        cylinderCenter[i] = (cylinderFieldBBox[2*i] + cylinderFieldBBox[2*i+1]) / 2.0f;
    }
    
    McVec3f cylEndPoints[2];
    cylEndPoints[0] = McVec3f(-cylinderLength/2.0f, 0.0f, 0.0f);
    cylEndPoints[1] = McVec3f( cylinderLength/2.0f, 0.0f, 0.0f);
    
    McVec3f transformedCylEndPoints[2];
    
    float phi, theta;
    const float phiMin   = 0.0f;
    const float phiMax   = 180.0f;
    const float thetaMin = 0.0f;
    const float thetaMax = 180.0f;
    float angleIncrement = portAngleIncrement.getValue();

    /* compute number of field required for a certain angle increment */
    int nFields = 
        int((phiMax  -phiMin)  /angleIncrement) * 
        int((thetaMax-thetaMin)/angleIncrement);
        
    int fieldIx = 0;
    cylinderFields.resize(nFields);
    cylinderMasks.resize(nFields);
    cylinderMaskElements.resize(nFields);
    cylinderOrientations.resize(nFields);

    int nVoxels = 
        cylinderFieldDims[0] * cylinderFieldDims[1] * cylinderFieldDims[2];
    McVec3f voxelSize(1.0f, 1.0f, 1.0f);
    
    ScanConvertCylinder scanConverter;
    McDArray<float> cylinderMaskTmp(nVoxels);
    McDArray<float> innerCylinderMaskTmp(nVoxels);
    float innerCylinderGrayValue = portInnerCylinderGrayValue.getValue();
    
    for ( phi=phiMin; phi<phiMax; phi+=angleIncrement ) {
        for ( theta=thetaMin; theta<thetaMax; theta+=angleIncrement ) {
            
            if ( progressInterface ) 
                progressInterface->setProgressValue(float(fieldIx)/float(nFields));

            cylinderFields[fieldIx].resize(nVoxels);
            cylinderOrientations[fieldIx] = McVec2f(phi, theta);
            
            McMat4f mat = computeTransformMatrix(phi, theta, 0.0, cylinderCenter);   
            mat.multVecMatrix(cylEndPoints[0], transformedCylEndPoints[0]);
            mat.multVecMatrix(cylEndPoints[1], transformedCylEndPoints[1]);

            /* scan convert outer cylinder */
            scanConverter.scanConvert(transformedCylEndPoints, 
                                      outerCylinderRadius, 
                                      portOuterCylinderGrayValue.getValue(),
                                      portBackgroundGrayValue.getValue(), 
                                      cylinderFieldBBox,
                                      cylinderFieldDims,
                                      voxelSize, 
                                      cylinderFields[fieldIx].dataPtr());

            /* scan convert inner cylinder */
            scanConverter.scanConvert(transformedCylEndPoints, 
                                      innerCylinderRadius, 
                                      1.0,
                                      -1.0, 
                                      cylinderFieldBBox,
                                      cylinderFieldDims,
                                      voxelSize, 
                                      innerCylinderMaskTmp.dataPtr());

            for ( i=0; i<innerCylinderMaskTmp.size(); i++ ) {
                if ( innerCylinderMaskTmp[i] > 0.0f ) {
                    cylinderFields[fieldIx][i] = innerCylinderGrayValue;
                } 
            }

            /* scan convert extended cylinder that will be used as mask */
            scanConverter.scanConvert(transformedCylEndPoints, 
                                      maskCylinderRadius, 
                                      1.0,
                                      -1.0, 
                                      cylinderFieldBBox,
                                      cylinderFieldDims,
                                      voxelSize, 
                                      cylinderMaskTmp.dataPtr());

            cylinderMasks[fieldIx].resize(nVoxels);
            cylinderMasks[fieldIx].unsetAll();
            cylinderMaskElements[fieldIx].clear();
            for ( i=0; i<cylinderMaskTmp.size(); i++ ) {
                if ( cylinderMaskTmp[i] > 0.0f ) {
                    cylinderMasks[fieldIx].set(i);
                    cylinderMaskElements[fieldIx].append(i);
                } 
            }
            
            fieldIx++;
        }
    }
    
    if ( progressInterface ) 
        progressInterface->stopWorking();
}

