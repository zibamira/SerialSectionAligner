/////////////////////////////////////////////////////////////////
// 
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef SCAN_CONVERT_CYLINDER_H
#define SCAN_CONVERT_CYLINDER_H

#include <mclib/McBitfield.h>
#include <mclib/McDArray.h>
#include <mclib/McVec3.h>
#include <mclib/McVec4i.h>

#include "api.h"

class HXTEMPLATEMATCHINGUTIL_API ScanConvertCylinder 
{
public:
    ScanConvertCylinder();
    
    void setScaleFactor(const McVec3f & scaleFactor);
    
    void scanConvert(const McVec3f   endPoints[2], 
                     const float     radius, 
                     const float     foregroundValue,
                     const float     backgroundValue, 
                     const float     bbox[6],
                     const int       dims[3],
                     const McVec3f & voxelSize,
                     float         * lattice);
    
    void scanConvert(const McVec3f   endPoints[2], 
                     const float     radius, 
                     const float     foregroundValue,
                     const float     backgroundValue, 
                     const float     bbox[6],
                     const int       dims[3],
                     const McVec3f & voxelSize,
                     const int       subDivision,
                     float         * lattice);

    void scanConvert(const McVec3f   endPoints[2], 
                     const float     radius, 
                     const float     bbox[6],
                     const int       dims[3],
                     const McVec3f & voxelSize,
                     McDArray<mculong> & voxelsInsideCylinder);

    void scanConvert(const McVec3f       endPoints[2], 
                     const float         radius, 
                     const float         bbox[6],
                     const int           dims[3],
                     const McVec3f     & voxelSize,
                     McDArray<McVec4i> & voxelsInsideCylinder);




    void getVoxels_longints(const McVec3f       endPoints[2], 
                               const float         radius2, 
                               const int           dims[3],
                               const McVec3i       offsetVec[6],
                               const McVec3f       coordsOffset[6],
                               McDArray<McVec3i> & voxelsInsideCylinder,
                               McDArray<mculong> & voxelIxs,
                               McDArray<McVec3f> & coords);

    void bresenham_longints(const McVec3f       endPoints[2], 
                               const float         radius,
                               const int           dims[3],
                               const float         bbox[6],
                               const float         voxelSize,
                               McDArray<McVec3i> & voxelsInsideCylinder,
                               McDArray<mculong> & voxelIxs,
                               McDArray<McVec3f> & coords,
                               McVec3i             offsetVec[6],
                               McVec3f             coordsOffset[6]);

    void scanConvert_longints(const McVec3f       endPoints[2], 
                                 const float         radius, 
                                 const float         bbox[6],
                                 const int           dims[3],
                                 const McVec3f     & voxelSize,
                                 McDArray<McVec3i> & voxelsInsideCylinder,
                                 McDArray<mculong> & voxelIxs);
    
private:
    void setForeground(const float               foregroundValue, 
                       const McDArray<McVec4i> & voxelsInsideCylinder, 
                       float                   * lattice);
    
    void setForeground(const McDArray<McVec4i> & voxelsInsideCylinder, 
                       const McDArray<float>   & grayValues,
                       float                   * lattice);

    void mapPercentageToGrayValue(const float             foregroundValue,
                                  const float             backgroundValue,
                                  const McDArray<float> & perCentOfVolumeInsideCyl,
                                  McDArray<float>       & grayValues);
        
    void setBackground(const float   backgroundValue, 
                       const int     nVoxels,
                       float       * lattice);
    
    void bresenham(const McVec3f       endPoints[2], 
                   const float         radius,
                   const int           dims[3],
                   const float         bbox[6],
                   const float         voxelSize,
                   McDArray<McVec4i> & points,
                   McDArray<McVec3f> & coords,
                   McVec4i             offsetVec[6],
                   McVec3f             coordsOffset[6]);
    
    void getVoxels(const McVec3f       endPoints[2], 
                   const float         radius2, 
                   const int           dims[3],
                   const McVec4i       offsetVec[6],
                   const McVec3f       coordsOffset[6],
                   McDArray<McVec4i> & points,
                   McDArray<McVec3f> & coords);

    int isInsideCylinder(const McVec3f & point, 
                         const McVec3f   endPoints[2],
                         const float     radius2, 
                         McVec3f         dirVec);

    void getPercentageOfVolumeInsideCylinder(const McVec3f             endPoints[2], 
                                             const float               radius, 
                                             const float               bbox[6],
                                             const McVec3f           & voxelSize,
                                             const int                 subDivision,
                                             const McDArray<McVec4i> & voxelsInsideCylinder, 
                                             McDArray<float>         & perCentOfVolumeInsideCyl);
    
    void getVoxelCenter(const float     bbox[6], 
                        const McVec3f & voxelSize,
                        const McVec4i & voxelIx,
                        McVec3f       & voxelCenter);
    
    int getNumSubVoxels(const McVec3f   endPoints[2],
                        const float     radius, 
                        McVec3f         dirVec,
                        const McVec3f & voxelCenter, 
                        const McVec3f & voxelSize, 
                        const int       subDivision);

    McVec3f scaleFactor;
};

#endif
