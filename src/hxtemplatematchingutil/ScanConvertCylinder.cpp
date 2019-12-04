/////////////////////////////////////////////////////////////////
// 
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#include "ScanConvertCylinder.h"
#include <QSet>

#include <mclib/McVec3i.h>
#include <cmath>

ScanConvertCylinder::ScanConvertCylinder()
{
    scaleFactor = McVec3f(1.0f, 1.0f, 1.0f);
}

void 
ScanConvertCylinder::setScaleFactor(const McVec3f & scaleFactor)
{
    this->scaleFactor = scaleFactor;
}

void 
ScanConvertCylinder::scanConvert(const McVec3f   endPoints[2], 
                                 const float     radius, 
                                 const float     foregroundValue,
                                 const float     backgroundValue, 
                                 const float     bbox[6],
                                 const int       dims[3],
                                 const McVec3f & voxelSize,
                                 float         * lattice) 
{
    McDArray<McVec4i> voxelsInsideCylinder;
    
    scanConvert(endPoints,
                radius,
                bbox,
                dims,
                voxelSize,
                voxelsInsideCylinder);
    
    int nVoxels = dims[0] * dims[1] * dims[2];

    setBackground(backgroundValue, 
                  nVoxels,
                  lattice);
    
    setForeground(foregroundValue,
                  voxelsInsideCylinder,
                  lattice);
}

void 
ScanConvertCylinder::scanConvert(const McVec3f   endPoints[2], 
                                 const float     radius, 
                                 const float     foregroundValue,
                                 const float     backgroundValue, 
                                 const float     bbox[6],
                                 const int       dims[3],
                                 const McVec3f & voxelSize,
                                 const int       subDivision,
                                 float         * lattice) 
{
    const float voxelDiagonal = 
        std::sqrt(  voxelSize[0]*voxelSize[0] 
                  + voxelSize[1]*voxelSize[1]
                  + voxelSize[2]*voxelSize[2]);
             
    const float extendedRadius = 
        radius + 0.5 * voxelDiagonal;

    McDArray<McVec4i> voxelsInsideCylinder;
    
    scanConvert(endPoints,
                extendedRadius,
                bbox,
                dims,
                voxelSize,
                voxelsInsideCylinder);
    
    int nVoxels = dims[0] * dims[1] * dims[2];

    setBackground(backgroundValue, 
                  nVoxels,
                  lattice);
    
    McDArray<float> perCentOfVolumeInsideCyl(voxelsInsideCylinder.size());
    getPercentageOfVolumeInsideCylinder(endPoints,
                                        radius,
                                        bbox, 
                                        voxelSize,
                                        subDivision,
                                        voxelsInsideCylinder, 
                                        perCentOfVolumeInsideCyl);
    
    McDArray<float> grayValues(voxelsInsideCylinder.size());
    // map percentage to gray value, i.e. interpolate between
    // foreground and background value
    mapPercentageToGrayValue(foregroundValue,
                             backgroundValue,
                             perCentOfVolumeInsideCyl,
                             grayValues);

    setForeground(voxelsInsideCylinder,
                  grayValues,
                  lattice);
}

void 
ScanConvertCylinder::scanConvert(const McVec3f   endPoints[2], 
                                 const float     radius, 
                                 const float     bbox[6],
                                 const int       dims[3],
                                 const McVec3f & voxelSize,
                                 McDArray<mculong> & voxelIxs)
{
    McDArray<McVec3i> voxelsInsideCylinder;

    scanConvert_longints(endPoints, 
                radius, 
                bbox,
                dims,
                voxelSize,
                voxelsInsideCylinder,
                voxelIxs);
    
   
}

void 
ScanConvertCylinder::scanConvert_longints(const McVec3f       endPoints[2], 
                                 const float         radius, 
                                 const float         bbox[6],
                                 const int           dims[3],
                                 const McVec3f     & voxelSize,
                                 McDArray<McVec3i> & voxelsInsideCylinder,
                                 McDArray<mculong> & voxelIxs)
{
    voxelsInsideCylinder.clear();
    McDArray<McVec3f> coords;
    McVec3i           offsetVec[6];
    McVec3f           coordsOffset[6];
    bresenham_longints(endPoints, 
              radius,
              dims,
              bbox,
              voxelSize[0],
              voxelsInsideCylinder,
              voxelIxs,
              coords,
              offsetVec,
              coordsOffset) ;

    getVoxels_longints(endPoints, 
              radius*radius, 
              dims,
              offsetVec,
              coordsOffset,
              voxelsInsideCylinder,
              voxelIxs,
              coords);
}

void 
ScanConvertCylinder::bresenham_longints(const McVec3f       endPoints[2], 
                               const float         radius,
                               const int           dims[3],
                               const float         bbox[6],
                               const float         voxelSize,
                               McDArray<McVec3i> & voxels,
                               McDArray<mculong> & voxelIxs,
                               McDArray<McVec3f> & coords,
                               McVec3i             offsetVec[6],
                               McVec3f             coordsOffset[6]) 
{
    /* find major axis */
    int major = 0, minor[2] = {0,1};
    float maxDist = std::abs(endPoints[0][0]-endPoints[1][0]);
    float tmpDist = std::abs(endPoints[0][1]-endPoints[1][1]);
    if (tmpDist > maxDist) {
        major = 1; maxDist = tmpDist;
    }
    tmpDist = fabs(endPoints[0][2]-endPoints[1][2]);
    if (tmpDist > maxDist) {
        major = 2; maxDist = tmpDist;
    }
    
    /* assign minor axes */
    switch (major) {
    case 0: // x-axis
        minor[0] = 1; minor[1] = 2; 
        break;
    case 1: // y-axis
        minor[0] = 0; minor[1] = 2; 
        break;
    case 2: // z-axis
        minor[0] = 0; minor[1] = 1; 
        break;
    }
    
    /* set offsets to be used later for breadth-first-search */
    offsetVec[0] = McVec3i( 1, 0, 0);
    offsetVec[1] = McVec3i(-1, 0, 0);
    offsetVec[2] = McVec3i( 0, 1, 0);
    offsetVec[3] = McVec3i( 0,-1, 0);
    offsetVec[4] = McVec3i( 0, 0, 1);
    offsetVec[5] = McVec3i( 0, 0,-1);
    coordsOffset[0] = McVec3f( voxelSize, 0.0, 0.0);
    coordsOffset[1] = McVec3f(-voxelSize, 0.0, 0.0);
    coordsOffset[2] = McVec3f( 0.0, voxelSize, 0.0);
    coordsOffset[3] = McVec3f( 0.0,-voxelSize, 0.0);
    coordsOffset[4] = McVec3f( 0.0, 0.0, voxelSize);
    coordsOffset[5] = McVec3f( 0.0, 0.0,-voxelSize);

    float majorDist = endPoints[1][major] - endPoints[0][major];
    
    McVec3f ep[2];
    if (majorDist > 0.0) {
        ep[0] = endPoints[0];
        ep[1] = endPoints[1];
    } else {
        ep[0] = endPoints[1];
        ep[1] = endPoints[0];
        majorDist *= -1.0;
    }
    
    /* get lengths of projections */
    float minorDist[2];
    minorDist[0] = ep[1][minor[0]] - ep[0][minor[0]];
    minorDist[1] = ep[1][minor[1]] - ep[0][minor[1]];

    /* compute slopes in projection planes */
    float m[2];
    m[0] = minorDist[0] / majorDist;
    m[1] = minorDist[1] / majorDist; 

    /* extend cylinder line such that the major coordinate lies in the
       grid plane orthogonal to the major axis */
    int id = int(((ep[0][major] - bbox[2*major]) / voxelSize) + 0.5);
    float factor = ((bbox[2*major]+float(id)*voxelSize) - ep[0][major]);
    ep[0][major]     = bbox[2*major]+float(id)*voxelSize;
    ep[0][minor[0]] += factor*m[0];
    ep[0][minor[1]] += factor*m[1];

    /* compute start node of grid */
    McVec3i curId;
    curId[major]    = (int)((ep[0][major]    + 0.5*voxelSize - bbox[2*major])    / voxelSize);
    curId[minor[0]] = (int)((ep[0][minor[0]] + 0.5*voxelSize - bbox[2*minor[0]]) / voxelSize);
    curId[minor[1]] = (int)((ep[0][minor[1]] + 0.5*voxelSize - bbox[2*minor[1]]) / voxelSize);
    
    /* compute start error */
    float error[2];
    error[0] = (ep[0][minor[0]] - 
                (bbox[2*minor[0]]+float(curId[minor[0]])*voxelSize))/voxelSize;
    error[1] = (ep[0][minor[1]] - 
                (bbox[2*minor[1]]+float(curId[minor[1]])*voxelSize))/voxelSize;

    McVec3f curCoord = McVec3f(bbox[0], bbox[2], bbox[4]);
    curCoord[0] += ((float)curId[0])*voxelSize;
    curCoord[1] += ((float)curId[1])*voxelSize;
    curCoord[2] += ((float)curId[2])*voxelSize;

    /* store first node */
    if (!(curId[0]<0 || curId[1]<0 || curId[2]<0 || 
          curId[0]>=dims[0] || curId[1]>=dims[1] || curId[2]>=dims[2])) {

        //curId[3] = curId[0] + dims[0]*(curId[1] + curId[2]*dims[1]);
        voxels.append(curId);
        voxelIxs.append((mculong)curId[0] + (mculong)dims[0]*(mculong)((mculong)curId[1] + (mculong)curId[2]*(mculong)dims[1]));
        coords.append(curCoord);
    }

    // compute number of steps, i.e. length of line
    int nSteps = (int)(fabs((ep[1][major]-ep[0][major])) / voxelSize) + 1;

    /* get voxels along cylinder line */
    float exact;
    int i;
    for (i=0; i<nSteps; i++) {
        curId[major]++;
        
        exact = float(curId[minor[0]]) + m[0] + error[0];
        curId[minor[0]] = int(exact + 0.5);
        error[0] = exact - float(curId[minor[0]]);
        
        exact = float(curId[minor[1]]) + m[1] + error[1];
        curId[minor[1]] = int(exact + 0.5);
        error[1] = exact - float(curId[minor[1]]);
        
        if (curId[0]<0 || curId[1]<0 || curId[2]<0 || 
            curId[0]>=dims[0] || curId[1]>=dims[1] || curId[2]>=dims[2]) 
            continue;
        
        voxelIxs.append((mculong)curId[0] + (mculong)dims[0]*(mculong)((mculong)curId[1] + (mculong)curId[2]*(mculong)dims[1]));
        voxels.append(curId);
        curCoord = McVec3f(bbox[0], bbox[2], bbox[4]);
        curCoord[0] += curId[0]*voxelSize;
        curCoord[1] += curId[1]*voxelSize;
        curCoord[2] += curId[2]*voxelSize;
        coords.append(curCoord);
    }
}

void 
ScanConvertCylinder::getVoxels_longints(const McVec3f       endPoints[2], 
                               const float         radius2, 
                               const int           dims[3],
                               const McVec3i       offsetVec[6],
                               const McVec3f       coordsOffset[6],
                               McDArray<McVec3i> & voxels,
                               McDArray<mculong> & voxelIxs,
                               McDArray<McVec3f> & coords)
{
    QSet<mculong> visited;

    const int numOffsets = 6;

    McVec3f dirVec = endPoints[1] - endPoints[0];
    dirVec.normalize();

    /* Mark voxels found by bresenham as visited first. */
    for ( int i = 0; i < voxels.size(); i++ ) {
        visited.insert(voxelIxs[i]);
    }

    McDArray<mclong> indexOffset(6);
    indexOffset[0] = 1;
    indexOffset[1] = -1;
    indexOffset[2] = dims[0];
    indexOffset[3] = -dims[0];
    indexOffset[4] = dims[0]*dims[1];
    indexOffset[5] = -dims[0]*dims[1];

    /* Search more voxels by breadth first search. */
    for ( int i = 0; i < voxels.size(); i++ ) {
        /* Check neighboring nodes. */
        for ( int j = 0; j < numOffsets; j++ ) {
            const McVec3i newNode = voxels[i] + offsetVec[j];
            if ( newNode[0] < 0 || newNode[1] < 0 || newNode[2] < 0 ||
                    newNode[0] >= dims[0] || newNode[1]>=dims[1] || newNode[2] >= dims[2] )
            {
                continue;
            }

            const mculong nodeId = (mclong)voxelIxs[i] + indexOffset[j];
            
            if ( visited.contains(nodeId) ) {
                continue;
            }

            const McVec3f newCoord = coords[i] + coordsOffset[j];
            if ( !isInsideCylinder(newCoord, endPoints, radius2, dirVec) ) {
                continue;
            }

            voxels.append(newNode);
            coords.append(newCoord);
            voxelIxs.append(nodeId);
            visited.insert(nodeId);
        }
    }
}



void 
ScanConvertCylinder::scanConvert(const McVec3f       endPoints[2], 
                                 const float         radius, 
                                 const float         bbox[6],
                                 const int           dims[3],
                                 const McVec3f     & voxelSize,
                                 McDArray<McVec4i> & voxelsInsideCylinder)
{
    voxelsInsideCylinder.clear();
    McDArray<McVec3f> coords;
    McVec4i           offsetVec[6];
    McVec3f           coordsOffset[6];
    bresenham(endPoints, 
              radius,
              dims,
              bbox,
              voxelSize[0],
              voxelsInsideCylinder,
              coords,
              offsetVec,
              coordsOffset) ;

    getVoxels(endPoints, 
              radius*radius, 
              dims,
              offsetVec,
              coordsOffset,
              voxelsInsideCylinder,
              coords);
}



void 
ScanConvertCylinder::setForeground(const float               foregroundValue, 
                                   const McDArray<McVec4i> & voxelsInsideCylinder, 
                                   float                   * lattice)
{
    int i;
    for ( i=0; i<voxelsInsideCylinder.size(); i++ ) {
        lattice[voxelsInsideCylinder[i][3]] = foregroundValue; 
    }
}

void 
ScanConvertCylinder::setForeground(const McDArray<McVec4i> & voxelsInsideCylinder, 
                                   const McDArray<float>   & grayValues,
                                   float                   * lattice)
{
    int i;
    for ( i=0; i<voxelsInsideCylinder.size(); i++ ) {
        lattice[voxelsInsideCylinder[i][3]] = grayValues[i]; 
    }
}

void 
ScanConvertCylinder::mapPercentageToGrayValue(const float             foregroundValue,
                                              const float             backgroundValue,
                                              const McDArray<float> & perCentOfVolumeInsideCyl,
                                              McDArray<float>       & grayValues)
{
    grayValues.resize(perCentOfVolumeInsideCyl.size());

    const float grayValueDiff = 
        foregroundValue - backgroundValue;
    
    int i;
    for ( i=0; i<grayValues.size(); i++ ) {
        grayValues[i] = backgroundValue + perCentOfVolumeInsideCyl[i] * grayValueDiff;
    }
}

void 
ScanConvertCylinder::setBackground(const float   backgroundValue, 
                                   const int     nVoxels,
                                   float       * lattice)
{
    int i;
    for ( i=0; i<nVoxels; i++ ) {
        *lattice++ = backgroundValue; 
    }
}

void 
ScanConvertCylinder::bresenham(const McVec3f       endPoints[2], 
                               const float         radius,
                               const int           dims[3],
                               const float         bbox[6],
                               const float         voxelSize,
                               McDArray<McVec4i> & voxels,
                               McDArray<McVec3f> & coords,
                               McVec4i             offsetVec[6],
                               McVec3f             coordsOffset[6]) 
{
    /* find major axis */
    int major = 0, minor[2] = {0,1};
    float maxDist = std::abs(endPoints[0][0]-endPoints[1][0]);
    float tmpDist = std::abs(endPoints[0][1]-endPoints[1][1]);
    if (tmpDist > maxDist) {
        major = 1; maxDist = tmpDist;
    }
    tmpDist = fabs(endPoints[0][2]-endPoints[1][2]);
    if (tmpDist > maxDist) {
        major = 2; maxDist = tmpDist;
    }
    
    /* assign minor axes */
    switch (major) {
    case 0: // x-axis
        minor[0] = 1; minor[1] = 2; 
        break;
    case 1: // y-axis
        minor[0] = 0; minor[1] = 2; 
        break;
    case 2: // z-axis
        minor[0] = 0; minor[1] = 1; 
        break;
    }
    
    /* set offsets to be used later for breadth-first-search */
    offsetVec[0] = McVec4i( 1, 0, 0, 1);
    offsetVec[1] = McVec4i(-1, 0, 0,-1);
    offsetVec[2] = McVec4i( 0, 1, 0, dims[0]);
    offsetVec[3] = McVec4i( 0,-1, 0,-dims[0]);
    offsetVec[4] = McVec4i( 0, 0, 1, dims[0]*dims[1]);
    offsetVec[5] = McVec4i( 0, 0,-1,-dims[0]*dims[1]);
    coordsOffset[0] = McVec3f( voxelSize, 0.0, 0.0);
    coordsOffset[1] = McVec3f(-voxelSize, 0.0, 0.0);
    coordsOffset[2] = McVec3f( 0.0, voxelSize, 0.0);
    coordsOffset[3] = McVec3f( 0.0,-voxelSize, 0.0);
    coordsOffset[4] = McVec3f( 0.0, 0.0, voxelSize);
    coordsOffset[5] = McVec3f( 0.0, 0.0,-voxelSize);

    float majorDist = endPoints[1][major] - endPoints[0][major];
    
    McVec3f ep[2];
    if (majorDist > 0.0) {
        ep[0] = endPoints[0];
        ep[1] = endPoints[1];
    } else {
        ep[0] = endPoints[1];
        ep[1] = endPoints[0];
        majorDist *= -1.0;
    }
    
    /* get lengths of projections */
    float minorDist[2];
    minorDist[0] = ep[1][minor[0]] - ep[0][minor[0]];
    minorDist[1] = ep[1][minor[1]] - ep[0][minor[1]];

    /* compute slopes in projection planes */
    float m[2];
    m[0] = minorDist[0] / majorDist;
    m[1] = minorDist[1] / majorDist; 

    /* extend cylinder line such that the major coordinate lies in the
       grid plane orthogonal to the major axis */
    int id = int(((ep[0][major] - bbox[2*major]) / voxelSize) + 0.5);
    float factor = ((bbox[2*major]+float(id)*voxelSize) - ep[0][major]);
    ep[0][major]     = bbox[2*major]+float(id)*voxelSize;
    ep[0][minor[0]] += factor*m[0];
    ep[0][minor[1]] += factor*m[1];

    /* compute start node of grid */
    McVec4i curId;
    curId[major]    = (int)((ep[0][major]    + 0.5*voxelSize - bbox[2*major])    / voxelSize);
    curId[minor[0]] = (int)((ep[0][minor[0]] + 0.5*voxelSize - bbox[2*minor[0]]) / voxelSize);
    curId[minor[1]] = (int)((ep[0][minor[1]] + 0.5*voxelSize - bbox[2*minor[1]]) / voxelSize);
    
    /* compute start error */
    float error[2];
    error[0] = (ep[0][minor[0]] - 
                (bbox[2*minor[0]]+float(curId[minor[0]])*voxelSize))/voxelSize;
    error[1] = (ep[0][minor[1]] - 
                (bbox[2*minor[1]]+float(curId[minor[1]])*voxelSize))/voxelSize;

    McVec3f curCoord = McVec3f(bbox[0], bbox[2], bbox[4]);
    curCoord[0] += ((float)curId[0])*voxelSize;
    curCoord[1] += ((float)curId[1])*voxelSize;
    curCoord[2] += ((float)curId[2])*voxelSize;

    /* store first node */
    if (!(curId[0]<0 || curId[1]<0 || curId[2]<0 || 
          curId[0]>=dims[0] || curId[1]>=dims[1] || curId[2]>=dims[2])) {

        curId[3] = curId[0] + dims[0]*(curId[1] + curId[2]*dims[1]);
        voxels.append(curId);
        coords.append(curCoord);
    }

    // compute number of steps, i.e. length of line
    int nSteps = (int)(fabs((ep[1][major]-ep[0][major])) / voxelSize) + 1;

    /* get voxels along cylinder line */
    float exact;
    int i;
    for (i=0; i<nSteps; i++) {
        curId[major]++;
        
        exact = float(curId[minor[0]]) + m[0] + error[0];
        curId[minor[0]] = int(exact + 0.5);
        error[0] = exact - float(curId[minor[0]]);
        
        exact = float(curId[minor[1]]) + m[1] + error[1];
        curId[minor[1]] = int(exact + 0.5);
        error[1] = exact - float(curId[minor[1]]);
        
        if (curId[0]<0 || curId[1]<0 || curId[2]<0 || 
            curId[0]>=dims[0] || curId[1]>=dims[1] || curId[2]>=dims[2]) 
            continue;
        
        curId[3] = curId[0] + dims[0]*(curId[1] + curId[2]*dims[1]);
        voxels.append(curId);
        curCoord = McVec3f(bbox[0], bbox[2], bbox[4]);
        curCoord[0] += curId[0]*voxelSize;
        curCoord[1] += curId[1]*voxelSize;
        curCoord[2] += curId[2]*voxelSize;
        coords.append(curCoord);
    }
}

void 
ScanConvertCylinder::getVoxels(const McVec3f       endPoints[2], 
                               const float         radius2, 
                               const int           dims[3],
                               const McVec4i       offsetVec[6],
                               const McVec3f       coordsOffset[6],
                               McDArray<McVec4i> & voxels,
                               McDArray<McVec3f> & coords)
{
    QSet<int> visited;

    const int numOffsets = 6;

    McVec3f dirVec = endPoints[1] - endPoints[0];
    dirVec.normalize();

    /* Mark voxels found by bresenham as visited first. */
    for ( int i = 0; i < voxels.size(); i++ ) {
        visited.insert(voxels[i][3]);
    }

    /* Search more voxels by breadth first search. */
    for ( int i = 0; i < voxels.size(); i++ ) {
        /* Check neighboring nodes. */
        for ( int j = 0; j < numOffsets; j++ ) {
            const McVec4i newNode = voxels[i] + offsetVec[j];
            if ( newNode[0] < 0 || newNode[1] < 0 || newNode[2] < 0 ||
                    newNode[0] >= dims[0] || newNode[1]>=dims[1] || newNode[2] >= dims[2] )
            {
                continue;
            }

            const int nodeId = newNode[3];
            if ( visited.contains(nodeId) ) {
                continue;
            }

            const McVec3f newCoord = coords[i] + coordsOffset[j];
            if ( !isInsideCylinder(newCoord, endPoints, radius2, dirVec) ) {
                continue;
            }

            voxels.append(newNode);
            coords.append(newCoord);
            visited.insert(nodeId);
        }
    }
}

int 
ScanConvertCylinder::isInsideCylinder(const McVec3f & point, 
                                      const McVec3f   endPoints[2],
                                      const float     radius2, 
                                      McVec3f         dirVec) 
{
    McVec3f vecToLine = endPoints[1] - point;

    // check whether the projection of point is outside the cylinder
    // at the one end of the cylinder
    if ( dirVec.dot(endPoints[1]-point)<0 ) 
        return 0;
    
    // check whether the projection of point is outside the cylinder
    // at the other end of the cylinder
    if ( dirVec.dot(point-endPoints[0])<0 ) 
        return 0;

    // move point according to cylinder scaling
    const McVec3f vecToPoint = point - endPoints[0];
    const float length = dirVec.dot(vecToPoint);
    const McVec3f vecToProjPoint = length * dirVec;
    McVec3f vecOrthToLine = vecToPoint - vecToProjPoint;
    vecOrthToLine = vecOrthToLine.compquot(scaleFactor);

    const McVec3f newPoint = endPoints[0] + vecToProjPoint + vecOrthToLine;
    vecToLine = endPoints[1] - newPoint;

    // check whether the point is outside the radius of the cylinder
    float dist2 = dirVec.cross(vecToLine).length2();
    if ( dist2 > radius2 ) 
        return 0;
    
    return 1;
}

void 
ScanConvertCylinder::getPercentageOfVolumeInsideCylinder(const McVec3f             endPoints[2], 
                                                         const float               radius, 
                                                         const float               bbox[6],
                                                         const McVec3f           & voxelSize,
                                                         const int                 subDivision,
                                                         const McDArray<McVec4i> & voxelsInsideCylinder, 
                                                         McDArray<float>         & perCentOfVolumeInsideCyl)
{
    McVec3f dirVec = endPoints[1] - endPoints[0];
    dirVec.normalize();
    
    int countSubVoxels;
    const int numSubVoxels = subDivision * subDivision * subDivision;
    
    McVec3f voxelCenter;

    int i;
    for ( i=0; i<voxelsInsideCylinder.size(); i++ ) {

        getVoxelCenter(bbox, 
                       voxelSize,
                       voxelsInsideCylinder[i],
                       voxelCenter);
        
        countSubVoxels = 
            getNumSubVoxels(endPoints, 
                            radius, 
                            dirVec, 
                            voxelCenter, 
                            voxelSize, 
                            subDivision);
        
        perCentOfVolumeInsideCyl[i] = 
            float(countSubVoxels)/float(numSubVoxels);
    }
}

void
ScanConvertCylinder::getVoxelCenter(const float     bbox[6], 
                                    const McVec3f & voxelSize,
                                    const McVec4i & voxelIx,
                                    McVec3f       & voxelCenter)
{
    int i;
    for ( i=0; i<3; i++ ) {
        voxelCenter[i] = bbox[2*i] + float(voxelIx[i]) * voxelSize[i];
    }
}

int 
ScanConvertCylinder::getNumSubVoxels(const McVec3f   endPoints[2],
                                     const float     radius, 
                                     McVec3f         dirVec,
                                     const McVec3f & voxelCenter, 
                                     const McVec3f & voxelSize, 
                                     const int       subDivision)
{
    if ( subDivision == 0 ) {

        const float radius2 = radius * radius;

        if ( isInsideCylinder(voxelCenter,
                              endPoints,
                              radius2, 
                              dirVec) ) {
            return 1;
        } 
          
        return 0;
    } else {
        /* Check whether the whole voxel is inside or outside the cylinder. 
           If either is the case, we don't need to further subdivide the voxel.
        */

        const float voxelDiagonal = 
            std::sqrt(  voxelSize[0]*voxelSize[0] 
                      + voxelSize[1]*voxelSize[1]
                      + voxelSize[2]*voxelSize[2]);

        const float decreasedRadius = 
            radius - 0.5 * voxelDiagonal;

        const float decreasedRadius2 =
            decreasedRadius > 0 ? decreasedRadius * decreasedRadius : 0.0f;

        McVec3f modifiedEndPoints[2];
        modifiedEndPoints[0] = endPoints[0] + ( 0.5 * voxelDiagonal * dirVec);
        modifiedEndPoints[1] = endPoints[1] - ( 0.5 * voxelDiagonal * dirVec);

        if ( isInsideCylinder(voxelCenter,
                              modifiedEndPoints,
                              decreasedRadius2, 
                              dirVec) ) 
        {
            /* voxel is completely inside the cylinder */
            int numSubVoxels = 1;
            for ( int i=0; i<subDivision; i++ ) {
                numSubVoxels *= 8;
            }   
            return numSubVoxels;
        } else {
            const float extendedRadius = 
                radius + 0.5 * voxelDiagonal;

            const float extendedRadius2 =
                extendedRadius * extendedRadius;

            modifiedEndPoints[0] = endPoints[0] - ( 0.5 * voxelDiagonal * dirVec);
            modifiedEndPoints[1] = endPoints[1] + ( 0.5 * voxelDiagonal * dirVec);

            if ( !isInsideCylinder(voxelCenter,
                                   modifiedEndPoints,
                                   extendedRadius2, 
                                   dirVec) ) 
            {
                /* voxel is completely outside the cylinder */
                return 0;
            } 
        }
    }

    McVec3f newVoxelSize = voxelSize / 2.0f;
    
    int numVoxels = 0;
    
    numVoxels +=
        getNumSubVoxels(endPoints, radius, dirVec, 
                        McVec3f(voxelCenter[0]-voxelSize[0]/4.0f,
                                voxelCenter[1]-voxelSize[1]/4.0f,
                                voxelCenter[2]-voxelSize[2]/4.0f),
                        newVoxelSize,
                        subDivision-1);

    numVoxels +=
        getNumSubVoxels(endPoints, radius, dirVec, 
                        McVec3f(voxelCenter[0]-voxelSize[0]/4.0f,
                                voxelCenter[1]-voxelSize[1]/4.0f,
                                voxelCenter[2]+voxelSize[2]/4.0f),
                        newVoxelSize,
                        subDivision-1);
                                
    numVoxels +=
        getNumSubVoxels(endPoints, radius, dirVec, 
                        McVec3f(voxelCenter[0]-voxelSize[0]/4.0f,
                                voxelCenter[1]+voxelSize[1]/4.0f,
                                voxelCenter[2]-voxelSize[2]/4.0f),
                        newVoxelSize,
                        subDivision-1);

    numVoxels +=
        getNumSubVoxels(endPoints, radius, dirVec, 
                        McVec3f(voxelCenter[0]-voxelSize[0]/4.0f,
                                voxelCenter[1]+voxelSize[1]/4.0f,
                                voxelCenter[2]+voxelSize[2]/4.0f),
                        newVoxelSize,
                        subDivision-1);

    numVoxels +=
        getNumSubVoxels(endPoints, radius, dirVec, 
                        McVec3f(voxelCenter[0]+voxelSize[0]/4.0f,
                                voxelCenter[1]-voxelSize[1]/4.0f,
                                voxelCenter[2]-voxelSize[2]/4.0f),
                        newVoxelSize,
                        subDivision-1);

    numVoxels +=
        getNumSubVoxels(endPoints, radius, dirVec, 
                        McVec3f(voxelCenter[0]+voxelSize[0]/4.0f,
                                voxelCenter[1]-voxelSize[1]/4.0f,
                                voxelCenter[2]+voxelSize[2]/4.0f),
                        newVoxelSize,
                        subDivision-1);

    numVoxels +=
        getNumSubVoxels(endPoints, radius, dirVec, 
                        McVec3f(voxelCenter[0]+voxelSize[0]/4.0f,
                                voxelCenter[1]+voxelSize[1]/4.0f,
                                voxelCenter[2]-voxelSize[2]/4.0f),
                        newVoxelSize,
                        subDivision-1);

    numVoxels +=
        getNumSubVoxels(endPoints, radius, dirVec, 
                        McVec3f(voxelCenter[0]+voxelSize[0]/4.0f,
                                voxelCenter[1]+voxelSize[1]/4.0f,
                                voxelCenter[2]+voxelSize[2]/4.0f),
                        newVoxelSize,
                        subDivision-1);

    return numVoxels;
}

