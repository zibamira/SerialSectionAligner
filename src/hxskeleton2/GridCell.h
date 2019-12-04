#ifndef HXGRIDCELLMODEL_GRID_CELL_H
#define HXGRIDCELLMODEL_GRID_CELL_H

#include <mclib/internal/McIndexer3D.h>
#include <mclib/internal/McAssert.h>



enum GridCellType {
      GCT_HIGHDIM = 1
    , GCT_0MANIFOLD = 2
    , GCT_1MANIFOLD
    , GCT_1BOUNDARY
    , GCT_2MANIFOLD
    , GCT_2BOUNDARY
    , GCT_JUNCTION
    , GCT_1XJUNCTION
};

class GridCell {
    public:
        GridCell (const int* p) {
            init (p);
        }

        GridCell (McVec3i p) {
            init (&p[0]);
        }

        GridCell (int i, int j, int k) {
            const int p[3] = {i, j, k};
            init (p);
        }

        int GetDimension () const {
            return mDim;
        }

        bool IsOpen (int d) const {
            return (mParities & (1 << d)) != 0;
        }

        bool IsClosed (int d) const {
            return (mParities & (1 << d)) == 0;
        }

        McVec3i GetPosition () const {
            return McVec3i (mPos[0], mPos[1], mPos[2]);
        }
       static const int dim2type[4];

    private:
        void init (const int* p) {
            mDim = 0;
            mParities = 0;
            for (int i = 0; i < 3; i++) {
                mPos[i] = p[i];
                if (mPos[i] % 2) {
                    mDim++;
                    mParities |= 1 << i;
                }
            }
        }

        int mPos[3];
        int mParities;
        int mDim;
};

inline bool AllDims2Nplus1 (const McDim3l& dims) {
    for (int i = 0; i < 3; i++) {
        if (dims[i] % 2 == 0) {
            return false;
        }
    }
    return true;
}

template <class T>
bool HasBorder2 (const McDim3l& dims, const T* const image) {
    McIndexer3DNotBoxed indexer (McDim3i(dims), 2);
    mclong idx;
    while (indexer.nextLong(idx)) {
        if (image[idx]) {
            return false;
        }
    }
    return true;
}

/**
    \pre HasBorder2 (dims, image)
    \pre AllDims2Nplus1 (dims)

    \param[out] if maxdim != 0, maxdim will be set to maximal dimension for well formed complexes

    \returns true if well formed cell complex
 */
template <class T>
bool WellFormedCellComplex (const McDim3l& dims, const T* const image, int* maxdim) {
    mcrequire (HasBorder2 (dims, image));

    int maxfounddim = -1;

    int offset[3] = {1, (int)dims[0], (int)dims[0] * (int)dims[1]};

    McIndexer3DBoxed indexer (McDim3i(dims), 2);
    mclong idx;
    McVec3i pos;
    while (indexer.nextLong(idx, pos)) {
        if (!image[idx]) {
            continue;
        }

        GridCell cell (pos);
        const int dim = cell.GetDimension();
        if (dim > maxfounddim) {
            maxfounddim = dim;
        }
        // check if all incident cells at open boundaries are included in the cell complex
        for (int d = 0; d < 3; d++) {
            if (cell.IsOpen (d)) {
                for (int delta = -1; delta < 2; delta += 2) {
                    const mclong neighboridx = idx + offset[d] * delta;
                    if (!image [neighboridx]) {
                        return false;
                    }
                }
            }
        }
    }
    if (maxdim) {
        *maxdim = maxfounddim;
    }
    return true;
}

/** Add cells until a well formed cell complex is formed. Highest value wins. 
    
    \pre HasBorder2 (dims, image)
 */
template <class T>
void CompleteCellComplex (const McDim3l& dims, T* const image) {
    mcrequire (HasBorder2 (dims, image));

    int offset[3] = {1, (int)dims[0], (int)dims[0] * (int)dims[1]};

    // 3 cells may propagate 3 times. TODO: optimize
    for (int i = 0; i < 3; i++) {
        McIndexer3DBoxed indexer (McDim3i(dims), 2);
        mclong idx;
        McVec3i pos;
        while (indexer.nextLong(idx, pos)) {
            if (!image[idx]) {
                continue;
            }

            GridCell cell (pos);
            // mark all incident cells at open boundaries
            for (int d = 0; d < 3; d++) {
                if (cell.IsOpen (d)) {
                    for (int delta = -1; delta < 2; delta += 2) {
                        const mclong neighboridx = idx + offset[d] * delta;
                        if (image [neighboridx] < image[idx]) {
                            image[neighboridx] = image[idx];
                        }
                    }
                }
            }
        }
    }
}

inline bool IdxMatchPosOffsets (const int* const offset, const mclong idx, const McVec3i pos) {
    mclong posXoffset2 = (mclong)pos[2] * (mclong)offset[2];
    mclong posXoffset1 = (mclong)pos[1] * (mclong)offset[1];
    mclong posXoffset0 = (mclong)pos[0] * (mclong)offset[0];
    return idx - posXoffset2 - posXoffset1 - posXoffset0 == 0;
}

template<class T>
int NumDimPlusOneIncidentCells (const int* offsets, const T* image, const mclong idx, const GridCell cell) {
    mcrequire (IdxMatchPosOffsets (offsets, idx, cell.GetPosition()));

    int count = 0;
    for (int d = 0; d < 3; d++) {
        if (cell.IsClosed (d)) {
            for (int delta = -1; delta < 2; delta += 2) {
                const mclong neighboridx = idx + (mclong)offsets[d] * delta;
                if (image[neighboridx]) {
                    count++;
                }
            }
        }
    }
    return count;
}

inline bool HasDimPlusOneIncidentCell (const int* const offset, const unsigned char* const image, const mclong idx, const GridCell cell) {
    mcrequire (IdxMatchPosOffsets (offset, idx, cell.GetPosition()));

    for (int d = 0; d < 3; d++) {
        if (cell.IsClosed (d)) {
            for (int delta = -1; delta < 2; delta += 2) {
                const mclong neighboridx = idx + (mclong)offset[d] * delta;
                if (image [neighboridx]) { // incident cell found
                    return true;
                }
            }
        }
    }
    return false;
}

inline McVec3i IdxToPos (const McDim3l& dims, mclong idx) {
    McVec3i res;

    res[2] = idx / (dims[0] * dims[1]);
    idx -= res[2] * mclong(dims[0]) * dims[1];
    res[1] = idx / dims[0];
    idx -= res[1] * dims[0];
    res[0] = idx;

    return res;
}

inline bool CellIdxHasDimension (const McDim3l& dims, mclong idx, int d) {
    McVec3i pos = IdxToPos (dims, idx);
    GridCell cell (pos);
    return cell.GetDimension() == d;
}

inline McVec3i IdxToPosOffsets (const int* offsets, mclong idx) {
    mcrequire (offsets[0] == 1);
    McVec3i res;

    res[2] = idx / offsets[2];
    idx -= res[2] * offsets[2];
    res[1] = idx / offsets[1];
    idx -= res[1] * offsets[1];
    res[0] = idx;

    return res;
}

inline bool CellIdxHasDimensionOffsets (const int* const offsets, mclong idx, int d) {
    McVec3i pos = IdxToPosOffsets (offsets, idx);
    GridCell cell (pos);
    return cell.GetDimension() == d;
}

inline bool CellNeighbors(const McDim3l& dims, const mclong low, const mclong high) {
    const McVec3i lowpos = IdxToPos(dims, low);
    const GridCell lowcell(lowpos);

    const McVec3i highpos = IdxToPos(dims, high);
    const GridCell highcell(highpos);

    if (highcell.GetDimension() != lowcell.GetDimension() + 1) {
        return false;
    }

    // verify that exactly one index differs by one
    bool match = false;
    const McVec3i diff = highpos - lowpos;
    for (int i = 0; i < 3 ; i++) {
        if (diff[i] == -1 || diff[i] == 1) {
            if (match) {
                return false;
            }
            match = true;
        }
    }
    return match;
}


#endif
