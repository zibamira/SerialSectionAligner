#include <hxspindle/FilamentGrid.h>




FilamentGrid::Cell::Cell()
    : mElements(10)
{
}




void FilamentGrid::Cell::add(int elementID, bool checkForDuplicates /* = false */)
{
    if (!checkForDuplicates)
    {
        mElements.push(elementID);
    }
    else
    {
        for (int i = 0; i < mElements.size(); ++i)
        {
            if (mElements[i] == elementID) return;
        }

        mElements.push(elementID);
    }
}




void FilamentGrid::Cell::clear()
{
    mElements.clear();
}




void FilamentGrid::Cell::getElements(McDArray< int >& elements) const
{
    for (int i = 0; i < mElements.size(); ++i)
    {
        bool found = false;

        for (int j = 0; j < elements.size(); ++j)
        {
            if (elements[j] == mElements[i])
            {
                found = true;
                break;
            }
        }

        if (!found) elements.push(mElements[i]);
    }
}




FilamentGrid::FilamentGrid(void)
{
}




FilamentGrid::FilamentGrid(const HxSpatialGraph& graph, float cellSize)
{
    initialize(graph, cellSize);
}




void FilamentGrid::add(const HxSpatialGraph& graph)
{
    int n = graph.getNumEdges();

    for (int i = 0; i < n; ++i)
    {
        int m = graph.getNumEdgePoints(i);

        for (int j = 1; j < m; ++j)
        {
            McVec3f p = graph.getEdgePoint(i, j - 1);
            McVec3f q = graph.getEdgePoint(i, j);

            add(p, q, i);
        }
    }
}





void FilamentGrid::add(const HxSpatialGraph& graph, const McDArray< int >& selection)
{
    int n = selection.size();

    for (int i = 0; i < n; ++i)
    {
        int m = graph.getNumEdgePoints(selection[i]);

        for (int j = 1; j < m; ++j)
        {
            McVec3f p = graph.getEdgePoint(selection[i], j - 1);
            McVec3f q = graph.getEdgePoint(selection[i], j);

            add(p, q, selection[i]);
        }
    }
}




void FilamentGrid::add(const McVec3f& segmentStart, const McVec3f& segmentEnd, int segmentID)
{
    McVec3i cell;
    McVec3f start = segmentStart;

    coordToCell(segmentStart, cell.i, cell.j, cell.k);

    //theMsg->printf("add segment: %d", segmentID);

    do
    {
        //theMsg->printf("... cell: %d %d %d", cell.i, cell.j, cell.k);

        int cellID = coordToCell(cell.i, cell.j, cell.k);

        if (cellID < 0) break;

        mCells[cellID].add(segmentID, true);
    }
    while(segCellIntersection(start, segmentEnd, cell));
}




void FilamentGrid::clear()
{
    for (int i = 0; i < mCells.size(); ++i)
    {
        mCells[i].clear();
    }
}




int FilamentGrid::coordToCell(int x, int y, int z) const
{
    return z * mDimensions[1] * mDimensions[0] + y * mDimensions[0] + x;
}




int FilamentGrid::coordToCell(const McVec3f& position) const
{
    McVec3f refPosition = position - mGridMin;

    int x = (int) (refPosition.x / mCellSize.x);
    int y = (int) (refPosition.y / mCellSize.y);
    int z = (int) (refPosition.z / mCellSize.z);

    if (!validCell(x, y, z)) return -1;

    return coordToCell(x, y, z);
}




void FilamentGrid::coordToCell(const McVec3f& position, int& x, int& y, int& z) const
{
    McVec3f refPoint = position - mGridMin;

    x = (int) (refPoint.x / mCellSize.x);
    y = (int) (refPoint.y / mCellSize.y);
    z = (int) (refPoint.z / mCellSize.z);
}




void FilamentGrid::getNeighbors(const McDArray< McVec3f >& filament, float radius, McDArray< int >& neighbors) const
{
    neighbors.clear();

    int n = filament.size();

    for (int i = 1; i < n; ++i)
    {
        McVec3f p = filament[i - 1];
        McVec3f q = filament[i];

        McVec3f boxMin(std::min(p.x, q.x), std::min(p.y, q.y), std::min(p.z, q.z));
        McVec3f boxMax(std::max(p.x, q.x), std::max(p.y, q.y), std::max(p.z, q.z));

        boxMin = boxMin - mGridMin;
        boxMax = boxMax - mGridMin;

        boxMin = boxMin - McVec3f(radius, radius, radius);
        boxMax = boxMax + McVec3f(radius, radius, radius);

        getNeighbors(boxMin, boxMax, neighbors);
    }
}




void FilamentGrid::getNeighbors(const McVec3f& boxMin, const McVec3f& boxMax, McDArray< int >& neighbors) const
{
    int sX, sY, sZ, eX, eY, eZ;

    if (mDimensions[0] <= 0 || mDimensions[1] <= 0 || mDimensions[2] <= 0) return;

    sX = (int)(boxMin.x / mCellSize.x);
    sY = (int)(boxMin.y / mCellSize.y);
    sZ = (int)(boxMin.z / mCellSize.z);
    eX = (int)(boxMax.x / mCellSize.x);
    eY = (int)(boxMax.y / mCellSize.y);
    eZ = (int)(boxMax.z / mCellSize.z);

    validateCell(sX, sY, sZ);
    validateCell(eX, eY, eZ);

    for (int i = sX; i <= eX; ++i)
    {
        for (int j = sY; j <= eY; ++j)
        {
            for (int k = sZ; k <= eZ; ++k)
            {
                int cellID = coordToCell(i, j, k);

                mCells[cellID].getElements(neighbors);
            }
        }
    }
}




void FilamentGrid::initialize(const HxSpatialGraph& graph, float cellSize)
{
    const McBox3f& box = graph.getBoundingBox();

    mGridMin.setValue(box[0], box[2], box[4]);
    mGridMax.setValue(box[1], box[3], box[5]);

    McVec3f diag = mGridMax - mGridMin;

    mDimensions[0] = 1 + (int) (diag.x / cellSize);
    mDimensions[1] = 1 + (int) (diag.y / cellSize);
    mDimensions[2] = 1 + (int) (diag.z / cellSize);

    McVec3f newDiag = McVec3f((float)(mDimensions[0]) * cellSize,
                              (float)(mDimensions[1]) * cellSize,
                              (float)(mDimensions[2]) * cellSize);

    McVec3f newMax = mGridMin + newDiag;
    McVec3f shift  = 0.5 * (mGridMax - newMax);

    mGridMin = mGridMin + shift;
    mGridMax = newMax   + shift;

    mCellSize.setValue(cellSize, cellSize, cellSize);

    mCells.resize(mDimensions[0] * mDimensions[1] * mDimensions[2]);

    theMsg->printf("init grid with %d %d %d cells", mDimensions[0], mDimensions[1], mDimensions[2]);

    clear();

    add(graph);
}




void FilamentGrid::initialize(const HxSpatialGraph& graph, const McDArray< int >& selection, float cellSize)
{
    const McBox3f& box = graph.getBoundingBox();

    mGridMin.setValue(box[0], box[2], box[4]);
    mGridMax.setValue(box[1], box[3], box[5]);

    McVec3f diag = mGridMax - mGridMin;

    mDimensions[0] = 1 + (int) (diag.x / cellSize);
    mDimensions[1] = 1 + (int) (diag.y / cellSize);
    mDimensions[2] = 1 + (int) (diag.z / cellSize);

    McVec3f newDiag = McVec3f((float)(mDimensions[0]) * cellSize,
                              (float)(mDimensions[1]) * cellSize,
                              (float)(mDimensions[2]) * cellSize);

    McVec3f newMax = mGridMin + newDiag;
    McVec3f shift  = 0.5 * (mGridMax - newMax);

    mGridMin = mGridMin + shift;
    mGridMax = newMax   + shift;

    mCellSize.setValue(cellSize, cellSize, cellSize);

    mCells.resize(mDimensions[0] * mDimensions[1] * mDimensions[2]);

    theMsg->printf("init grid with %d %d %d cells", mDimensions[0], mDimensions[1], mDimensions[2]);

    clear();

    add(graph, selection);
}




bool FilamentGrid::segCellIntersection(McVec3f& segmentStart, const McVec3f& segmentEnd, McVec3i& cell) const
{
    double times[3];
    int    signes[3];

    McVec3d cellMin = McVec3d(mGridMin.x + (float) cell.i * mCellSize.x,
                              mGridMin.y + (float) cell.j * mCellSize.y,
                              mGridMin.z + (float) cell.k * mCellSize.z);

    McVec3d cellMax   = cellMin + McVec3d(mCellSize);
    McVec3d bounds[2] = {cellMin, cellMax};

    McVec3d segDir = McVec3d(segmentEnd) - McVec3d(segmentStart);
            segDir.normalize();
    McVec3d segDirI = McVec3d(1.0 / segDir.x, 1.0 / segDir.y, 1.0 / segDir.z);

    signes[0] = segDirI.x > 0.0 ? 1 : 0;
    signes[1] = segDirI.y > 0.0 ? 1 : 0;
    signes[2] = segDirI.z > 0.0 ? 1 : 0;

    times[0] = (bounds[signes[0]].x - segmentStart.x) * segDirI.x;
    times[1] = (bounds[signes[1]].y - segmentStart.y) * segDirI.y;
    times[2] = (bounds[signes[2]].z - segmentStart.z) * segDirI.z;

    if (times[0] < times[1] && times[0] < times[2])
    {
        if (signes[0] > 0) cell.i++;
        else               cell.i--;
    }
    else if (times[1] < times[0] && times[1] < times[2])
    {
        if (signes[1] > 0) cell.j++;
        else               cell.j--;

        times[0] = times[1];
    }
    else
    {
        if (signes[2] > 0) cell.k++;
        else               cell.k--;

        times[0] = times[2];
    }

    if (times[0] > (segmentEnd - segmentStart).length()) return false;

    segmentStart = segmentStart + McVec3f(times[0] * segDir);

    return true;
}




void FilamentGrid::validateCell(int& x, int& y, int& z) const
{
    x = (std::min)((std::max)(0, x), mDimensions[0] - 1);
    y = (std::min)((std::max)(0, y), mDimensions[1] - 1);
    z = (std::min)((std::max)(0, z), mDimensions[2] - 1);
}




bool FilamentGrid::validCell(int x, int y, int z) const
{
    return x >= 0 && x < mDimensions[0] && y >= 0 && y < mDimensions[1] && z >= 0 && z < mDimensions[2];
}



