#ifndef HXTEMPLATEMATCHINGUTIL__MCUTIL_H
#define HXTEMPLATEMATCHINGUTIL__MCUTIL_H

#include <mclib/McDArray.h>
#include <mclib/McVec3i.h>
class McBitfield;

#include "api.h"

namespace mcutil {

/** Normalizes @c values such that their mean value is 0.0 and
    their standard deviation is 1.0.

    \param values The values to be normalized in place.

*/
HXTEMPLATEMATCHINGUTIL_API void normalizeValues(McDArray<float>* values);


namespace detail {

inline mculong index1(mclong i, mclong j, mclong k, mclong d0, mclong d1)
{
    return (k * d1 + j) * d0 + i;
}

inline McVec3i index3(mculong idx, mclong d0, mclong d1)
{
    McVec3i idx3;

    const mculong sizeXYPlane = d0 * d1;
    idx3[2] = int(idx / sizeXYPlane);
    idx -= sizeXYPlane * idx3[2];
    idx3[1] = int(idx / d0);
    idx -= d0 * idx3[1];
    idx3[0] = idx;

    return idx3;
}

}  // namespace detail


/// Compute linear index in grid of dims from ijk.
inline mculong index1(McVec3i ijk, const int* dims)
{
    return detail::index1(ijk[0], ijk[1], ijk[2], dims[0], dims[1]);
}

/// Compute ijk in grid of dims from linear index.
inline McVec3i index3(mculong idx, const int* dims)
{
    return detail::index3(idx, dims[0], dims[1]);
}


/// Compute linear index in grid of dims from ijk.  Consider using index1().
inline mculong getPosIx(const McVec3i& pos, const int dims[3])
{
    return detail::index1(pos[0], pos[1], pos[2], dims[0], dims[1]);
}

/// Compute ijk in grid of dims from linear index.  Consider using index3().
inline void getGridPos(mculong posIx, const int dims[3], McVec3i& gridPos)
{
    gridPos = detail::index3(posIx, dims[0], dims[1]);
}


/// Compute linear index in grid of dims from ijk.  Consider using index1().
inline mculong getPosIx(const McVec3i& pos, const McVec3i& dims)
{
    return detail::index1(pos[0], pos[1], pos[2], dims[0], dims[1]);
}

/// Compute ijk in grid of dims from linear index.  Consider using index3().
inline void getGridPos(mculong posIx, const McVec3i& dims, McVec3i& gridPos)
{
    gridPos = detail::index3(posIx, dims[0], dims[1]);
}


/// Compute linear index in grid of dims from ijk.  Consider using index1().
inline mculong latticePos(int i, int j, int k, const int* dims)
{
    return detail::index1(i, j, k, dims[0], dims[1]);
}

/// Compute ijk in grid of dims from linear index.  Consider using index3().
inline void latticeCoord(int& i, int& j, int& k, mculong pos, const int* dims)
{
    const McVec3i ijk = detail::index3(pos, dims[0], dims[1]);
    i = ijk[0];
    j = ijk[1];
    k = ijk[2];
}


///
HXTEMPLATEMATCHINGUTIL_API McDArray<float> GetSelection(
        const McDArray<float>& InArray,
        const McBitfield& selection);

///
HXTEMPLATEMATCHINGUTIL_API void InsertSelection(
        const McDArray<float>& modiArray,
        const McBitfield& selection,
        McDArray<float>& InArray);

}  // namespace mcutil

#endif
