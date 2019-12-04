#include "HxSerialSectionStack.h"

#include <hxcore/HxBase.h>
#include <hxcore/internal/HxReadAmiraMesh.h>
#include <hxcore/internal/HxWorkArea.h>
#include <hxcore/HxMessage.h>
#include <hxcore/HxObjectPool.h>
#include <hxcore/HxFileDialog.h>
#include <hxcore/HxResource.h>
#include <hxfield/HxCoordType.h>
#include <hxfield/HxUniformScalarField3.h>
#include <hxspindle/SpindleTools.h>
#include <mclib/internal/McComparators.h>
#include <mclib/McException.h>
#include <mclib/internal/McSorter.h>
#include <qdebug.h>

#ifdef _OPENMP
    #include <omp.h>
#endif



HX_INIT_CLASS (HxSerialSectionStack, HxSpatialData);




namespace
{
    class IndexedEdge
    {
        public:


            void combine(IndexedEdge& edge)
            {
                int   n = mPoints.size() - 1;
                int   m = edge.mPoints.size() - 1;
                float d[4];

                d[0] = (mPoints[0] - edge.mPoints[0]).length();
                d[1] = (mPoints[0] - edge.mPoints[m]).length();
                d[2] = (mPoints[n] - edge.mPoints[0]).length();
                d[3] = (mPoints[n] - edge.mPoints[m]).length();

                // start - start connection

                if (d[0] <= d[1] && d[0] <= d[2] && d[0] <= d[3])
                {
                    for (int i = 0; i < edge.mPoints.size(); ++i)
                    {
                        mPoints.insert(0, edge.mPoints[i]);
                        mIndices.insert(0, edge.mIndices[i]);
                    }
                }

                // start - end connection

                else if (d[1] <= d[0] && d[1] <= d[2] && d[1] <= d[3])
                {
                    for (int i = 0; i < edge.mPoints.size(); ++i)
                    {
                        mPoints.insert(i, edge.mPoints[i]);
                        mIndices.insert(i, edge.mIndices[i]);
                    }
                }

                // end - start connection

                else if (d[2] <= d[0] && d[2] <= d[1] && d[2] <= d[3])
                {
                    for (int i = 0; i < edge.mPoints.size(); ++i)
                    {
                        mPoints.push(edge.mPoints[i]);
                        mIndices.push(edge.mIndices[i]);
                    }
                }

                // end - end connection

                else
                {
                    for (int i = edge.mPoints.size() - 1; i >= 0; --i)
                    {
                        mPoints.push(edge.mPoints[i]);
                        mIndices.push(edge.mIndices[i]);
                    }
                }
            }




            void resize(int size)
            {
                mPoints.resize(size);
                mIndices.resize(size);
            }


        public:


            McDArray< McVec3f > mPoints;
            McDArray< int     > mIndices;
    };
}




/**
    Computes the stack of filaments based on the previously warped
    filaments of the single sections and the matchings between the
    filaments.
*/
static HxSpatialGraph* buildStack(const McDArray< McHandle<HxSpatialGraph> >&       graphs,
                                  const McDArray< HxSerialSectionStack::Matching >& matchings,
                                  const McDArray< McVec3f >&                        zRanges,
                                  const float                                       voxelSizeZ)
{
    HxSpatialGraph* stack = HxSpatialGraph::createInstance();

    int   numGraphs = graphs.size();
    int   numInterfaces = matchings.size();
    float z = 0.0;

    // extract edges with section ids

    McDArray< IndexedEdge >      edges;
    McDArray< McDArray < int > > edgeMap(numGraphs);

    for (int i = 0; i < numGraphs; ++i)
    {
        if (!graphs[i]) continue;

        float zMin = zRanges[i].x;
        float zMax = zRanges[i].y;
        float zShift = (z - zMin) + zRanges[i].z;
        int   numEdges = graphs[i]->getNumEdges();

        edgeMap[i].resize(numEdges);

        for (int j = 0; j < numEdges; ++j)
        {
            int         numPoints = graphs[i]->getNumEdgePoints(j);
            IndexedEdge edge;

            edge.resize(numPoints);
            edgeMap[i][j] = edges.size();

            for (int k = 0; k < numPoints; ++k)
            {
                McVec3f p = graphs[i]->getEdgePoint(j, k);
                p.z += zShift;

                edge.mPoints[k] = p;
                edge.mIndices[k] = i;
            }

            edges.push(edge);
        }

        z += zMax - zMin;
        z += voxelSizeZ;
    }

    // connect edges by given mapping

    McDArray< bool > finalEdges(edges.size());

    finalEdges.fill(true);

    for (int i = 0; i < numInterfaces; ++i)
    {
        const HxSerialSectionStack::Matching& matching = matchings[i];

        for (int j = 0; j < matching.getNumMatches(); ++j)
        {
            int edgeSource = edgeMap[i][matching.get(j).i];
            int edgeTarget = edgeMap[i + 1][matching.get(j).k];

            if (edgeSource == edgeTarget)
            {
                theMsg->printf("loop problem: section %d MT %d with section %d MT %d", i, matching.get(j).i, i + 1, matching.get(j).k);
            }
            else
            {
                edges[edgeSource].combine(edges[edgeTarget]);
            }

            finalEdges[edgeTarget] = false;
            edgeMap[i + 1][matching.get(j).k] = edgeSource;
        }
    }

    // build graph

    for (int i = 0; i < edges.size(); ++i)
    {
        if (!finalEdges[i]) continue;

        stack->addVertex(edges[i].mPoints[0]);
        stack->addVertex(edges[i].mPoints[edges[i].mPoints.size() - 1]);
        stack->addEdge(stack->getNumVertices() - 2, stack->getNumVertices() - 1, edges[i].mPoints);
    }

    // section attribute

    PointAttribute* sectionAtt = stack->addPointAttribute("sectionID", McPrimType::MC_INT32, 1);

    int edgeID = 0;

    for (int i = 0; i < edges.size(); ++i)
    {
        if (!finalEdges[i]) continue;

        for (int j = 0; j < edges[i].mPoints.size(); ++j)
        {
            sectionAtt->setIntDataAtPoint(edges[i].mIndices[j], edgeID, j);
        }
        edgeID++;
    }

    return stack;
}




static void evalTomogram(HxUniformScalarField3* stack, HxUniformScalarField3* tomogram, const McDArray< McVec2f >& points, const int zPosition)
{
    if (!stack || !tomogram) return;

    theWorkArea->startWorking("warp section");

    const int   dimStackX = stack->lattice().getDims()[0];
    const int   dimTomogramZ = tomogram->lattice().getDims()[2];
    const int   numPoints = points.size();
    const float voxelSize = stack->getVoxelSize(0, 0, 0).z;

    // create a tomogram location for each thread

    #ifdef _OPENMP
        int numThreads = omp_get_max_threads();
    #else
        int numThreads = 1;
    #endif

    McDArray< HxLocation3* > tomogramLoc(numThreads);

    for (int j = 0; j < numThreads; ++j)
    {
        tomogramLoc[j] = tomogram->createLocation();
    }

    // check which point in the x-y-plane of the stack is outside the tomogram

    const McBox3f& tomogramBox = tomogram->getBoundingBox();

    McDArray< bool > inside(numPoints);

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int id = 0; id < numPoints; ++id)
    {
        if (points[id].x < tomogramBox[0] || points[id].x > tomogramBox[1] ||
            points[id].y < tomogramBox[2] || points[id].y > tomogramBox[3])
        {
            inside[id] = false;
        }
        else
        {
            inside[id] = true;
        }
    }

    // do expensive eval for all z-slices of the tomogram

    for (int z = 0; z < dimTomogramZ; ++z)
    {
        float zValue = (float)z * voxelSize + tomogramBox[4];

        #ifdef _OPENMP
            #pragma omp parallel for
        #endif

        for (int id = 0; id < numPoints; ++id)
        {
            #ifdef _OPENMP
                int threadID = omp_get_thread_num();
            #else
                int threadID = 0;
            #endif

            int x = id % dimStackX;
            int y = id / dimStackX;

            float value = 0.0f;

            if (inside[id])
            {
                tomogramLoc[threadID]->set(McVec3f(points[id].x, points[id].y, zValue));

                if (!tomogram->eval(*tomogramLoc[threadID], &value)) value = 0.0f;
            }
            stack->lattice().set(x, y, z + zPosition, &value);
        }
        theWorkArea->setProgressValue((float)(z + 1) / (float)dimTomogramZ);
    }

    // clean locations

    for (int j = 0; j < numThreads; ++j)
    {
        delete tomogramLoc[j];
    }

    theWorkArea->stopWorking();
}




/**
    Computes the 2D positions in a z-slice with a certain
    voxel size and a 2D shift.
*/
static void initPlane(McDArray< McVec2f >& points, const int dimX, const int dimY, const McVec3f& voxelSize, const McVec2f& shift)
{
    int numSlicePoints = dimX * dimY;

    points.resize(numSlicePoints);

    for (int y = 0; y < dimY; ++y)
    {
        for (int x = 0; x < dimX; ++x)
        {
            points[y * dimX + x].setValue(shift.x + (float)x * voxelSize.x, shift.y + (float)y * voxelSize.y);
        }
    }
}




/**
    Warps the x-y coordinates of a single 3D point using
    moving least squares.
*/
static McVec3f warpPoint(const MovingLeastSquares& mls, const McVec3f& point)
{
    McVec2d vertex2D = McVec2d(point.x, point.y);
    McVec2d vertex2DWarped = mls.interpolate(vertex2D);

    return McVec3f((float) vertex2DWarped.x, (float) vertex2DWarped.y, point.z);
}




/**
    Warps a set of point using moving leas squares.
    The fast option uses a grid and only the 10 closest
    control points for the warping.
*/
static void warpPoints(MovingLeastSquares& mls, McDArray< McVec2f >& points, bool fast)
{
    if (fast) mls.initializeGrid();

    int numPoints = points.size();
    int numDone = 0;

    theWorkArea->startWorking("warping");

    #ifdef _OPENMP
        numPoints /= omp_get_max_threads();
        #pragma omp parallel for
    #endif

    for (int id = 0; id < points.size(); ++id)
    {
        McVec2d wp;

        if (fast)
            wp = mls.interpolateFast(McVec2d(points[id].x, points[id].y), 10);
        else
            wp = mls.interpolate(McVec2d(points[id].x, points[id].y));

        points[id].setValue((float) wp.x, (float) wp.y);

        // show progress

        #ifdef _OPENMP
            int threadID = omp_get_thread_num();
        #else
            int threadID = 0;
        #endif

        if (threadID == 0)
        {
            numDone++;
            theWorkArea->setProgressValue(std::min(1.0f, (float)(numDone) / (float) numPoints));
        }
    }
    theWorkArea->stopWorking();
}




/**
    Warping of the a spatial graph using moving leas squares.
    Only the x-y coordinates are effected.
*/
static void warpSpatialGraph(const MovingLeastSquares& mls, HxSpatialGraph& graph)
{
    // warp vertices

    int numVertices = graph.getNumVertices();

    for (int i = 0; i < numVertices; ++i)
    {
        graph.setVertexCoords(i, warpPoint(mls, graph.getVertexCoords(i)));
    }

    // warp edges

    for (int i = 0; i < graph.getNumEdges(); ++i)
    {
        int numPoints = graph.getNumEdgePoints(i);

        McDArray< McVec3f > points(numPoints);

        for (int j = 0; j < numPoints; ++j)
        {
            points[j] = warpPoint(mls, graph.getEdgePoint(i, j));
        }

        graph.setEdgePoints(i, points);
    }
}




/**
    Computes the final bounding box of the full stack.
*/
static McBox3f computeBox(const McDArray< HxSerialSectionStack::Landmarks >& landmarks,
                          const McDArray< McBox3f >&                         boundingBoxes,
                                int                                          centerSectionID)
{

    McBox3f box(FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX);

    int n = boundingBoxes.size();

    McDArray< McDArray< McVec2f > > points(n);

    for (int i = 0; i < n; ++i)
    {
        McVec3f boxMin = boundingBoxes[i].getMin();
        McVec3f boxMax = boundingBoxes[i].getMax();

        points[i].resize(4);

        points[i][0].setValue(boxMin.x, boxMin.y);
        points[i][1].setValue(boxMax.x, boxMin.y);
        points[i][2].setValue(boxMax.x, boxMax.y);
        points[i][3].setValue(boxMin.x, boxMax.y);

        box[4] = std::min(box[4], boxMin.z);
        box[5] = std::max(box[5], boxMax.z);
    }

    int m = n - 1;

    MovingLeastSquares mls;

    // warp above start section

    for (int i = m - 1; i >= centerSectionID; --i)
    {
        landmarks[i].setupMLS(mls);
        mls.swapLandmarks();

        for (int j = i + 1; j < n; ++j)
        {
            warpPoints(mls, points[j], false);
        }
    }

    // warp below start section

    for (int i = 0; i < centerSectionID; ++i)
    {
        landmarks[i].setupMLS(mls);

        for (int j = i; j >= 0; --j)
        {
            warpPoints(mls, points[j], false);
        }
    }

    // compute bounding box

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            box[0] = std::min(box[0], points[i][j].x);
            box[2] = std::min(box[2], points[i][j].y);
            box[1] = std::max(box[1], points[i][j].x);
            box[3] = std::max(box[3], points[i][j].y);
        }
    }

    return box;
}





static HxSerialSectionStack::SaveInfo si [] =
{
    { "AmiraMesh ASCII", "am", (SaveMethod) &HxSerialSectionStack::saveAmiraMesh },
    { "AmiraMesh Binary", "am", (SaveMethod) &HxSerialSectionStack::saveAmiraMesh },
    { 0, 0, 0 }
};




HxSerialSectionStack::Section::Section()
    : mFilaments(0)
    , mImageFileName("")
{
}




McHandle< HxUniformScalarField3 > HxSerialSectionStack::Section::getImage() const
{
    const int numObjects = theObjectPool->getNodeList().size();

    theObjectPool->setHideNewModules(true);

    HxResource::loadData(mImageFileName);

    theObjectPool->setHideNewModules(false);

    const int numObjectsLoaded = theObjectPool->getNodeList().size() - numObjects;

    McHandle< HxUniformScalarField3 > image = 0;

    QString exceptionMsg;

    if (numObjectsLoaded <= 0)
    {
        return image;
    }
    else
    {
        for (int i = 0; i < numObjectsLoaded; ++i)
        {
            HxObject* obj = theObjectPool->getNodeList()[numObjects + i];

            if (obj->isOfType(HxUniformScalarField3::getClassTypeId()))
            {
                image = dynamic_cast< HxUniformScalarField3 *>(obj);

                break;
            }
        }
    }

    // remove data from pool

    for (int i = 0; i < numObjectsLoaded; ++i)
    {
        theObjectPool->removeObject(theObjectPool->getNodeList().last());
    }

    return image;
}




void HxSerialSectionStack::Landmarks::addMark(const McVec2f& point1, const McVec2f& point2, const SettingType type)
{
    mMarks[0].push(point1);
    mMarks[1].push(point2);
    mTypes.push(type);
}




void HxSerialSectionStack::Landmarks::addMark(const McVec2d& point1, const McVec2d& point2, const SettingType type)
{
    mMarks[0].push(McVec2f((float) point1.x, (float) point1.y));
    mMarks[1].push(McVec2f((float) point2.x, (float) point2.y));
    mTypes.push(type);
}




void HxSerialSectionStack::Landmarks::clear()
{
    mMarks[0].clear();
    mMarks[1].clear();
    mTypes.clear();
}




void HxSerialSectionStack::Landmarks::deleteMark(int markID)
{
    mMarks[0].remove(markID);
    mMarks[1].remove(markID);
    mTypes.remove(markID);
}




void HxSerialSectionStack::Landmarks::deleteMarks(const SettingType type)
{
    int n = mMarks[0].size();

    for (int i = n - 1; i >= 0; --i)
    {
        if (mTypes[i] == type)
        {
            deleteMark(i);
        }
    }
}




const McVec2f& HxSerialSectionStack::Landmarks::getMark(int markID, int section) const
{
    return mMarks[section][markID];
}




const McDArray< McVec2f >& HxSerialSectionStack::Landmarks::getMarks(int section) const
{
    return mMarks[section];
}




int HxSerialSectionStack::Landmarks::getNumMarks() const
{
    return mMarks[0].size();
}




int HxSerialSectionStack::Landmarks::getNumMarks(HxSerialSectionStack::SettingType type) const
{
    int n = mMarks[0].size();
    int m = 0;

    for (int i = 0; i < n; ++i)
    {
        if (mTypes[i] == type) m++;
    }

    return m;
}




HxSerialSectionStack::SettingType HxSerialSectionStack::Landmarks::getType(int markID) const
{
    return mTypes[markID];
}




void HxSerialSectionStack::Landmarks::setMark(const McVec2f& position, int markID, int section)
{
    mMarks[section][markID] = position;
}




void HxSerialSectionStack::Landmarks::setNumMarks(const int numMarks)
{
    mMarks[0].resize(numMarks);
    mMarks[1].resize(numMarks);
    mTypes.resize(numMarks);
}




void HxSerialSectionStack::Landmarks::setType(const int markID, const SettingType type)
{
    mTypes[markID] = type;
}




void HxSerialSectionStack::Landmarks::setupMLS(MovingLeastSquares& mls) const
{
    int n = getNumMarks();

    McDArray< McVec2d > source(n);
    McDArray< McVec2d > target(n);

    for (int i = 0; i < n; ++i)
    {
        source[i].setValue(getMark(i, 0).x, getMark(i, 0).y);
        target[i].setValue(getMark(i, 1).x, getMark(i, 1).y);
    }

    mls.setLandmarks(source, target);
}




void HxSerialSectionStack::Landmarks::transform(mtalign::FacingPointSets& points) const
{
    int n = getNumMarks();

    McDArray< McVec2d > source(n);
    McDArray< McVec2d > target(n);

    for (int i = 0; i < n; ++i)
    {
        source[i].setValue(getMarks(0)[i].x, getMarks(0)[i].y);
        target[i].setValue(getMarks(1)[i].x, getMarks(1)[i].y);
    }

    // transform MT end points with directions of target section to source section

    MovingLeastSquares mls;

    mls.setLandmarks(target, source);

    int numPoints = points.trans.positions.size();

    for (int i = 0; i < numPoints; ++i)
    {
        McVec2d p1(points.trans.positions[i].x, points.trans.positions[i].y);
        McVec2d p2(points.trans.directions[i].x + p1.x, points.trans.directions[i].y + p1.y);

        McVec2d tp1 = mls.interpolate(p1);
        McVec2d tp2 = mls.interpolate(p2);

        points.trans.positions[i].x = tp1.x;
        points.trans.positions[i].y = tp1.y;
        points.trans.directions[i].x = tp2.x - tp1.x;
        points.trans.directions[i].y = tp2.y - tp1.y;
        points.trans.directions[i].normalize();
    }
}




void HxSerialSectionStack::Landmarks::transformInv(mtalign::WarpResult& warpResult) const
{
    int n = getNumMarks();

    McDArray< McVec2d > source(n);
    McDArray< McVec2d > target(n);

    for (int i = 0; i < n; ++i)
    {
        source[i].setValue(getMarks(0)[i].x, getMarks(0)[i].y);
        target[i].setValue(getMarks(1)[i].x, getMarks(1)[i].y);
    }

    // transform CPD result points from source section to target section

    MovingLeastSquares mls;

    mls.setLandmarks(source, target);

    int numPoints = warpResult.mlsParams.ps.size();

    for (int i = 0; i < numPoints; ++i)
    {
        McVec2d tp = mls.interpolate(warpResult.mlsParams.ps[i]);

        warpResult.mlsParams.ps[i] = tp;
    }
}




void HxSerialSectionStack::Matching::add(const McVec2i& filamentEnd1, const McVec2i& filamentEnd2)
{
    McVec3f color = computeColor(MANUAL);

    add(filamentEnd1, filamentEnd2, color, MANUAL);
}




void HxSerialSectionStack::Matching::add(const McVec2i& filamentEnd1, const McVec2i& filamentEnd2, const McVec3f& color, const SettingType type)
{
    int n = mMatches.size();

    for (int i = 0; i < n; ++i)
    {
        // do not allow branchings

        if ((mMatches[i].i == filamentEnd1.x &&
             mMatches[i].j == filamentEnd1.y) ||
            (mMatches[i].k == filamentEnd2.x &&
             mMatches[i].l == filamentEnd2.y))
        {
            return;
        }

        // do not allow cycles

        if ((mMatches[i].i == filamentEnd1.x) &&
            (mMatches[i].k == filamentEnd2.x))
        {
            return;
        }
    }

    mMatches.push(McVec4i(filamentEnd1.x, filamentEnd1.y, filamentEnd2.x, filamentEnd2.y));
    mMatchColors.push(color);
    mMatchTypes.push(type);

    if (type == MANUAL)
    {
        setState(MATCHED, filamentEnd1.x, filamentEnd1.y, 0);
        setState(MATCHED, filamentEnd2.x, filamentEnd2.y, 1);
    }
    if (type == TEMPORARY)
    {
        setState(MATCHED_AUTOMATIC, filamentEnd1.x, filamentEnd1.y, 0);
        setState(MATCHED_AUTOMATIC, filamentEnd2.x, filamentEnd2.y, 1);
    }
}




void HxSerialSectionStack::Matching::clear()
{
    mMatches.clear();
    mMatchColors.clear();
    mMatchTypes.clear();
}




McVec3f HxSerialSectionStack::Matching::computeColor(HxSerialSectionStack::SettingType type) const
{
    McVec3f color(0.2, 0.2, 0.2);

    McVec3f redColors[6];

    redColors[0].setValue(254.0 / 255.0, 178.0 / 255.0, 76.0 / 255.0);
    redColors[1].setValue(253.0 / 255.0, 141.0 / 255.0, 60.0 / 255.0);
    redColors[2].setValue(252.0 / 255.0,  78.0 / 255.0, 42.0 / 255.0);
    redColors[3].setValue(227.0 / 255.0,  26.0 / 255.0, 28.0 / 255.0);
    redColors[4].setValue(189.0 / 255.0,   0.0 / 255.0, 38.0 / 255.0);
    redColors[5].setValue(128.0 / 255.0,   0.0 / 255.0, 38.0 / 255.0);

    McVec3f greenColors[6];

    greenColors[0].setValue(173.0 / 255.0, 221.0 / 255.0, 142.0 / 255.0);
    greenColors[1].setValue(120.0 / 255.0, 198.0 / 255.0, 121.0 / 255.0);
    greenColors[2].setValue( 65.0 / 255.0, 171.0 / 255.0,  93.0 / 255.0);
    greenColors[3].setValue( 35.0 / 255.0, 132.0 / 255.0,  67.0 / 255.0);
    greenColors[4].setValue(  0.0 / 255.0, 104.0 / 255.0,  55.0 / 255.0);
    greenColors[5].setValue(  0.0 / 255.0,  69.0 / 255.0,  41.0 / 255.0);

    if (type == MANUAL)
    {
        color = greenColors[rand() % 6];
    }
    else if (type == AUTOMATIC || type == TEMPORARY)
    {
        color = redColors[rand() % 6];
    }

    return color;
}




const McVec4i& HxSerialSectionStack::Matching::get(int matchID) const
{
    return mMatches[matchID];
}




const McVec3f& HxSerialSectionStack::Matching::getColor(int matchID) const
{
    return mMatchColors[matchID];
}




int HxSerialSectionStack::Matching::getNumMatches() const
{
    return mMatches.size();
}




int HxSerialSectionStack::Matching::getNumStates(int section) const
{
    return mFilamentEndState[section].size();
}




HxSerialSectionStack::FilamentEndState HxSerialSectionStack::Matching::getState(int filamentID, int end, int section) const
{
    return mFilamentEndState[section][filamentID * 2 + end];
}




HxSerialSectionStack::SettingType HxSerialSectionStack::Matching::getType(int matchID) const
{
    return mMatchTypes[matchID];
}




bool HxSerialSectionStack::Matching::isLandmarkForbidden(int filamentID, int end, int section) const
{
    return mFilamentEndForbidLandmarks[section][filamentID * 2 + end];
}




bool HxSerialSectionStack::Matching::isLandmarkForbidden(int matchID) const
{
    return isLandmarkForbidden(mMatches[matchID].i, mMatches[matchID].j, 0) ||
           isLandmarkForbidden(mMatches[matchID].k, mMatches[matchID].l, 1);
}




void HxSerialSectionStack::Matching::print()
{
    for (int i = 0; i < mMatches.size(); ++i)
    {
        theMsg->printf("%d, %d <-> %d, %d", mMatches[i].i, mMatches[i].j, mMatches[i].k, mMatches[i].l);
    }
}




void HxSerialSectionStack::Matching::remove(int matchID)
{
    McVec4i match = mMatches[matchID];

    mMatches.remove(matchID);
    mMatchColors.remove(matchID);
    mMatchTypes.remove(matchID);

    setState(UNDEFINED, match.i, match.j, 0);
    setState(UNDEFINED, match.k, match.l, 1);
}




void HxSerialSectionStack::Matching::remove(HxSerialSectionStack::SettingType type)
{
    for (int i = 0; i < mMatches.size(); ++i)
    {
        if (mMatchTypes[i] == type)
        {
            remove(i);
            i--;
        }
    }
}




void HxSerialSectionStack::Matching::set(const McDArray< McDArray< McVec2i > >& selection, const mtalign::MatchingPGM& matching)
{
    remove(TEMPORARY);

    int n = matching.matchedRefPointIds.size();

    for (int i = 0; i < n; ++i)
    {
        McVec2i sourceMT = selection[0][matching.matchedRefPointIds[i]];
        McVec2i targetMT = selection[1][matching.matchedTransPointIds[i]];

        McVec3f color = computeColor(TEMPORARY);

        add(sourceMT, targetMT, color, TEMPORARY);
    }
}




void HxSerialSectionStack::Matching::setNumStates(int section, int numStates)
{
    mFilamentEndState[section].resize(numStates);
    mFilamentEndState[section].fill(UNDEFINED);

    mFilamentEndForbidLandmarks[section].resize(numStates);
    mFilamentEndForbidLandmarks[section].fill(false);
}




void HxSerialSectionStack::Matching::setState(HxSerialSectionStack::FilamentEndState state, int filamentID, int end, int section)
{
    mFilamentEndState[section][filamentID * 2 + end] = state;
}




void HxSerialSectionStack::Matching::setType(HxSerialSectionStack::SettingType type, int matchID)
{
    if (mMatchTypes[matchID] == type) return;

    if ((mMatchTypes[matchID] == TEMPORARY && type == AUTOMATIC) ||
        (mMatchTypes[matchID] == AUTOMATIC && type == TEMPORARY))
    {
        mMatchTypes[matchID] = type;
    }
    else
    {
        mMatchTypes[matchID] = type;
        mMatchColors[matchID] = computeColor(type);
    }
}




void HxSerialSectionStack::Matching::switchForbiddenState(int filamentID, int end, int section)
{
    mFilamentEndForbidLandmarks[section][filamentID * 2 + end] = !mFilamentEndForbidLandmarks[section][filamentID * 2 + end];
}




void HxSerialSectionStack::Matching::updateEndStates(const HxSpatialGraph* graph1, const HxSpatialGraph* graph2)
{
    if (!graph1)
    {
        setNumStates(0, 0);
    }
    else if (graph1->getNumEdges() * 2 != mFilamentEndState[0].size())
    {
        setNumStates(0, graph1->getNumEdges() * 2);
    }

    if (!graph2)
    {
        setNumStates(1, 0);
    }
    else if (graph2->getNumEdges() * 2 != mFilamentEndState[1].size())
    {
        setNumStates(1, graph2->getNumEdges() * 2);
    }
}




HxSerialSectionStack::HxSerialSectionStack () 
    : portAction (this, "action", tr("Action"), 2)
    , portInfo (this, "info", tr("info"))
{
        saveInfo = si;

        portAction.setLabel (0, "add files");
        portAction.setLabel (1, "flip stack");

        portInfo.printf ("");
}




HxSerialSectionStack::~HxSerialSectionStack ()
{
}




int HxSerialSectionStack::addSection()
{
    Section section;
    const int sectionNumber = mSections.append(section);

    if (mSectionInterfaces.size() != mSections.size() - 1)
    {
        mSectionInterfaces.resize(mSections.size() - 1);
    }

    return sectionNumber;
}




void HxSerialSectionStack::addSectionData(McHandle<HxSpatialData> data, const int sectionID)
{
    mcassert(data);

    McHandle<HxSpatialGraph>        graph = dynamic_cast<HxSpatialGraph*>(data.ptr());
    McHandle<HxUniformScalarField3> image = dynamic_cast<HxUniformScalarField3*>(data.ptr());

    mcassert(graph || image);

    if (graph)
    {
        setFilaments(graph, sectionID);
    }
    else if (image)
    {
        setImage(image, sectionID);
    }
}




void HxSerialSectionStack::compute()
{
    bool up = false;

    // add serial sections
    if (portAction.wasHit (0))
    {
        HxFileDialog::FileNameAndFormatList files = theFileDialog->getOpenFileNames();
        if (!files.isEmpty())
        {
            for (int i = 0; i < files.size(); i++)
            {
                McHandle<HxSpatialData> data = loadSectionData(qPrintable(files[i].first));
            }

            up = true;
        }
    }

    // updates parameters

    if (up)
    {
        updateInfo();
        touch();
        touchConnections();
    }


    // change stack oder

    if (portAction.wasHit(1))
    {
        int n = mSections.size();
        int m = n / 2;

        for (int i = 0; i < m; ++i)
        {
            Section tmp(mSections[i]);

            mSections[i]         = mSections[n - i - 1];
            mSections[n - i - 1] = tmp;
        }

        updateInfo();
        touch();
        touchConnections();
    }
}




void HxSerialSectionStack::createStack(HxSpatialGraph& filaments, HxUniformScalarField3& field, float voxelSizeXY, int centerSectionID)
{
    int numSections = mSections.size();
    int numInterfaces = numSections - 1;

    if (numSections == 0) return;

    float   voxelSizeZ = mSections[0].mImageBoundingBox.getVoxelSize(mSections[0].mImageDimensions).z;
    McVec3f voxelSize(voxelSizeXY, voxelSizeXY, voxelSizeZ);

    McDArray< McVec3f > zRanges(numSections);
    McDArray< int >     zDims(numSections);

    for (int i = 0; i < numSections; ++i)
    {
        const McBox3f box  = mSections[i].mImageBoundingBox;
        const McDim3l dims = mSections[i].mImageDimensions;

        McBox3f graphBox = mSections[i].mFilaments ? mSections[i].mFilaments->getBoundingBox() : box;

        zRanges[i].setValue(box[4], box[5], graphBox[4] - box[4]);
        zDims[i] = dims[2];
    }

    // duplicate graphs for the warping

    McDArray< McHandle< HxSpatialGraph > > graphs(numSections);

    for (int i = 0; i < numSections; ++i)
    {
        graphs[i] = HxSpatialGraph::createInstance();

        if (mSections[i].mFilaments)
        {
            graphs[i]->copyFrom(mSections[i].mFilaments);
        }
    }

    // duplicate landmarks to warp them

    McDArray< Landmarks > landmarks(numInterfaces);
    McDArray< Matching > matchings(numInterfaces);

    for (int i = 0; i < numInterfaces; ++i)
    {
        landmarks[i] = mSectionInterfaces[i].mLandmarks;
        matchings[i] = mSectionInterfaces[i].mMatching;
    }

    MovingLeastSquares mls;

    // warp above start section

    for (int i = numInterfaces - 1; i >= centerSectionID; --i)
    {
        landmarks[i].setupMLS(mls);

        mls.swapLandmarks();

        for (int j = i + 1; j < numSections; ++j)
        {
            warpSpatialGraph(mls, *graphs[j]);
        }
    }

    // warp below start section

    for (int i = 0; i < centerSectionID; ++i)
    {
        landmarks[i].setupMLS(mls);

        for (int j = i; j >= 0; --j)
        {
            warpSpatialGraph(mls, *graphs[j]);
        }
    }

    filaments.copyFrom(buildStack(graphs, matchings, zRanges, voxelSize.z));

    // warp fields and build stack

    // compute x-y dimensions based on full stack

    McBox3f box = filaments.getBoundingBox();

    if (filaments.getNumEdges() == 0)
    {
        McDArray< McBox3f > boxes(numSections);

        for (int i = 0; i < numSections; ++i)
        {
            boxes[i] = mSections[i].mImageBoundingBox;
        }

        box = computeBox(landmarks, boxes, centerSectionID);
    }

    int dims[3] = { 0, 0, 0 };

    dims[0] = (int)((box[1] - box[0]) / voxelSize.x) + 1;
    dims[1] = (int)((box[3] - box[2]) / voxelSize.y) + 1;

    for (int i = 0; i < numSections; ++i)
    {
        dims[2] += zDims[i];
    }

    theMsg->printf("final res: %d %d %d", dims[0], dims[1], dims[2]);

    // build tomogram stack

    float tomogramBox[6];

    tomogramBox[0] = box[0];
    tomogramBox[1] = box[0] + (dims[0] - 1) * voxelSize.x;
    tomogramBox[2] = box[2];
    tomogramBox[3] = box[2] + (dims[1] - 1) * voxelSize.y;
    tomogramBox[4] = 0.0;
    tomogramBox[5] = (dims[2] - 1) * voxelSize.z;

    field.lattice().init(dims, 1, McPrimType::MC_UINT8, C_UNIFORM, 0);
    field.lattice().setBoundingBox(tomogramBox);

    // warp above start section

    McDArray< McVec2f > points;

    initPlane(points, dims[0], dims[1], voxelSize, McVec2f(box[0], box[2]));

    int zStart = 0;

    for (int i = 0; i < centerSectionID; ++i) zStart += zDims[i];

    for (int i = centerSectionID; i < numSections; ++i)
    {
        McHandle< HxUniformScalarField3 > section = mSections[i].getImage();

        evalTomogram(&field, section, points, zStart);

        zStart += zDims[i];

        // update warping for new section

        if (i >= numInterfaces) continue;

        landmarks[i].setupMLS(mls);
        warpPoints(mls, points, false);
    }

    // warp below start section

    initPlane(points, dims[0], dims[1], voxelSize, McVec2f(box[0], box[2]));

    zStart = 0;

    for (int i = 0; i < centerSectionID - 1; ++i) zStart += zDims[i];

    for (int i = centerSectionID - 1; i >= 0; --i)
    {
        // update warping for new section

        landmarks[i].setupMLS(mls);
        mls.swapLandmarks();
        warpPoints(mls, points, false);

        // get values for warped section

        McHandle< HxUniformScalarField3 > section = mSections[i].getImage();

        evalTomogram(&field, section, points, zStart);

        if (i > 0) zStart -= zDims[i - 1];
    }

    field.fire();
}





void HxSerialSectionStack::getBoundingBox (float b[6]) const
{
    b[0] = b[2] = b[4] = 0.0;
    b[1] = b[3] = b[5] = 1.0;

    return;
}




HxSerialSectionStack::SectionInterface& HxSerialSectionStack::getSectionInterface(int sectionInterface)
{
    return mSectionInterfaces[sectionInterface];
}




int HxSerialSectionStack::getNumSections()
{
    return mSections.size();
}




const HxSerialSectionStack::Section& HxSerialSectionStack::getSection(int sectionID) const
{
    return mSections[sectionID];
}




McHandle<HxSpatialData> HxSerialSectionStack::loadSectionData(const char* filename)
{
    const int numObjects = theObjectPool->getNodeList().size();

    theObjectPool->setHideNewModules(true);

    // do not remove this module

    setFlag( CAN_BE_REMOVED_ALL, false );

    HxResource::loadData (QString::fromLatin1(filename));

    setFlag( CAN_BE_REMOVED_ALL, true );

    theObjectPool->setHideNewModules(false);

    const int numObjectsLoaded = theObjectPool->getNodeList().size() - numObjects;

    McHandle<HxSpatialData> resultData = 0;

    QString exceptionMsg;

    if (numObjectsLoaded <= 0)
    {
        exceptionMsg = QString("HxSerialSectionStack: no data loaded from file %1.").arg(QString::fromLatin1(filename));
    }
    else
    {
        for (int i = 0; i < numObjectsLoaded; ++i)
        {
            HxObject* obj = theObjectPool->getNodeList()[numObjects + i];

            if (obj->isOfType(HxSpatialGraph::getClassTypeId()) ||
                obj->isOfType(HxUniformScalarField3::getClassTypeId()))
            {
                mcassert(obj->isOfType(HxSpatialData::getClassTypeId()));

                resultData = dynamic_cast<HxSpatialData*>(obj);

                QString tmpStr;

                if (!resultData->parameters.findString("Filename", tmpStr))
                {
                    resultData->parameters.set("Filename", QString::fromLatin1(filename));
                }

                addSectionData(resultData, -1);
            }
        }
    }

    // remove data from pool

    for (int i = 0; i < numObjectsLoaded; ++i)
    {
        theObjectPool->removeObject(theObjectPool->getNodeList().last());
    }

    if (!resultData)
    {
        throw McException(exceptionMsg);
    }

    return resultData;
}




int HxSerialSectionStack::parse(Tcl_Interp* t, int argc, char **argv)
{
    const char* cmd = argv[1];

    if (CMD("addSection"))
    {
        ASSERTARG (3);

        McHandle<HxSpatialData> data = loadSectionData(argv[2]);

        if (data == 0)
        {
            return TCL_ERROR;
        }

        addSectionData(data, -1);
        updateInfo();
        touch();
        touchConnections();
    }
    else if (CMD("getSection"))
    {
        ASSERTARG (3);
        McHandle< HxUniformScalarField3 > data = getSection(atoi (argv[2])).getImage();

        if (!data)
        {
            return TCL_ERROR;
        }
        else
        {
            theObjectPool->addObject(data);
            Tcl_VaSetResult(t, "%s", qPrintable(data->getLabel ()));
        }
    }
    else if (CMD("removeSection"))
    {
        ASSERTARG (3);
        removeSection(atoi (argv[2]));
    }
    else if (CMD("getNumSections"))
    {
        ASSERTARG (2);
        Tcl_VaSetResult(t, "%d", getNumSections());
    }
    else
    {
        return HxSpatialData::parse (t, argc, argv);
    }
    return TCL_OK;
}




int HxSerialSectionStack::readAmiraMesh(AmiraMesh* m, const char* filename)
{
    HxParamBundle* sectionsPB = m->parameters.getBundle("Sections", 0);

    if (!sectionsPB)
    {
        throw McException("Could not find Sections parameter");
    }

    HxSerialSectionStack* stack = new HxSerialSectionStack;

    QString   fileName = QString::fromLatin1(filename);
    QFileInfo file(fileName);
    QDir      filePath(file.path());

    int numSections = sectionsPB->getNumberOfBundles();

    for (int i = 0; i < numSections; ++i)
    {
        stack->addSection();

        HxParamBundle* sectionPB = sectionsPB->getBundle(i);
        QString sectionName(sectionPB->getName());
        McString sectionNumberStr(qPrintable(sectionName), int(strlen("Section")), 4);
        bool ok;
        const int sectionNumber = sectionNumberStr.toInt(ok);

        if (!ok)
        {
            throw McException(QString("Could not determine section number in parameter %1").arg(sectionPB->getName()));
        }

        QString imageFN, reconstructionFN;

        if (sectionPB->findString("Image", imageFN))
        {
            stack->mSections[i].mImageFileName = filePath.absoluteFilePath(imageFN);
        }

        if (sectionPB->findString("Reconstruction", reconstructionFN))
        {
            McHandle<HxSpatialData> reconstruction = stack->loadSectionData(qPrintable(filePath.absoluteFilePath(reconstructionFN)));
            stack->addSectionData(reconstruction, sectionNumber);
        }
    }

    // load sections

    AmiraMesh::Data* imageBoxesData = m->findData("SECTIONS", HxFLOAT, 6, "ImageBoxes");
    AmiraMesh::Data* imageDimsData = m->findData("SECTIONS", HxINT32, 3, "ImageDimensions");


    float* imageBoxes = (float*)imageBoxesData->getDataPtr();
    int*   imageDims = (int*)imageDimsData->getDataPtr();

    for (int i = 0; i < numSections; ++i)
    {
        stack->mSections[i].mImageBoundingBox[0] = imageBoxes[i * 6 + 0];
        stack->mSections[i].mImageBoundingBox[1] = imageBoxes[i * 6 + 1];
        stack->mSections[i].mImageBoundingBox[2] = imageBoxes[i * 6 + 2];
        stack->mSections[i].mImageBoundingBox[3] = imageBoxes[i * 6 + 3];
        stack->mSections[i].mImageBoundingBox[4] = imageBoxes[i * 6 + 4];
        stack->mSections[i].mImageBoundingBox[5] = imageBoxes[i * 6 + 5];
        stack->mSections[i].mImageDimensions[0] = imageDims[i * 3 + 0];
        stack->mSections[i].mImageDimensions[1] = imageDims[i * 3 + 1];
        stack->mSections[i].mImageDimensions[2] = imageDims[i * 3 + 2];
    }

    // load section changes

    AmiraMesh::Data* numLandmarksData = m->findData("SECTION_CHANGES", HxINT32, 1, "Landmarks");
    AmiraMesh::Data* numMatchingsData = m->findData("SECTION_CHANGES", HxINT32, 1, "Matchings");
    AmiraMesh::Data* numEndStatesData = m->findData("SECTION_CHANGES", HxINT32, 2, "MicrotubuleEnds");

    if (!numLandmarksData)
    {
        throw McException(QString("Error: Cannot read SerialSectionStack %1. No number of landmarks data.").arg(QString::fromLatin1(filename)));
    }
    if (!numEndStatesData)
    {
        throw McException(QString("Error: Cannot read SerialSectionStack %1. No number of ends data.").arg(QString::fromLatin1(filename)));
    }

    AmiraMesh::Data* landmarkData3D = m->findData("LANDMARKS", HxFLOAT, 6, "Coordinates");
    AmiraMesh::Data* landmarkData2D = m->findData("LANDMARKS", HxFLOAT, 4, "Coordinates");
    AmiraMesh::Data* landmarkTypesData = m->findData("LANDMARKS", HxINT32, 1, "Types");
    AmiraMesh::Data* matchingData = m->findData("MATCHINGS", HxINT32, 4, "Matches");
    AmiraMesh::Data* matchingColorData = m->findData("MATCHINGS", HxFLOAT, 3, "Colors");
    AmiraMesh::Data* matchingTypesData = m->findData("MATCHINGS", HxINT32, 1, "Types");
    AmiraMesh::Data* endStatesData = m->findData("MICROTUBULE_ENDS", HxINT32, 1, "States");

    if ((!landmarkData2D && !landmarkData3D) || !landmarkTypesData ||
        !matchingData || !matchingColorData || !matchingTypesData)
    {
        throw McException(QString("Error: Cannot read SerialSectionStack %1. Some attribute is missing.").arg(QString::fromLatin1(filename)));
    }

    const int numInterfaces = numLandmarksData->getLocation()->getDimensions()[0];
    const int numMatchings = matchingData->getLocation()->getDimensions()[0];

    int*     numMarks = (int*)numLandmarksData->getDataPtr();
    McVec2f* landmarks2D = 0;
    McVec3f* landmarks3D = 0;
    int*     landmarkTypes = (int*)landmarkTypesData->getDataPtr();

    if (landmarkData2D) landmarks2D = (McVec2f*)landmarkData2D->getDataPtr();
    if (landmarkData3D) landmarks3D = (McVec3f*)landmarkData3D->getDataPtr();

    int*     numMatching = (int*)numMatchingsData->getDataPtr();
    McVec2i* matchings = (McVec2i*)matchingData->getDataPtr();
    McVec3f* matchingColors = (McVec3f*)matchingColorData->getDataPtr();
    int*     matchingTypes = (int*)matchingTypesData->getDataPtr();

    McVec2i* numEnds = (McVec2i*)numEndStatesData->getDataPtr();
    int*     endStates = (int*)endStatesData->getDataPtr();

    mcassert(numInterfaces == stack->mSections.size() - 1);

    int idx = 0;
    int jdx = 0;
    int kdx = 0;

    for (int i = 0; i < numInterfaces; ++i)
    {
        // landmarks

        stack->mSectionInterfaces[i].mLandmarks.setNumMarks(numMarks[i]);

        for (int k = 0; k < numMarks[i]; ++k)
        {
            stack->mSectionInterfaces[i].mLandmarks.setType(k, (SettingType)landmarkTypes[idx / 2]);

            if (landmarks2D)
            {
                stack->mSectionInterfaces[i].mLandmarks.setMark(landmarks2D[idx++], k, 0);
                stack->mSectionInterfaces[i].mLandmarks.setMark(landmarks2D[idx++], k, 1);
            }
            else
            {
                stack->mSectionInterfaces[i].mLandmarks.setMark(SpindleTools::vec2(landmarks3D[idx++]), k, 0);
                stack->mSectionInterfaces[i].mLandmarks.setMark(SpindleTools::vec2(landmarks3D[idx++]), k, 1);
            }
        }

        // end states

        stack->mSectionInterfaces[i].mMatching.setNumStates(0, numEnds[i].x);
        stack->mSectionInterfaces[i].mMatching.setNumStates(1, numEnds[i].y);

        for (int j = 0; j < numEnds[i].x; ++j)
        {
            FilamentEndState state = (FilamentEndState)(endStates[kdx] & 0x7FFF);
            bool             forbid = (bool)(endStates[kdx] & 0x8000);

            stack->mSectionInterfaces[i].mMatching.setState(state, j / 2, j % 2, 0);

            if (stack->mSectionInterfaces[i].mMatching.isLandmarkForbidden(j / 2, j % 2, 0) != forbid)
            {
                stack->mSectionInterfaces[i].mMatching.switchForbiddenState(j / 2, j % 2, 0);
            }

            kdx++;
        }
        for (int j = 0; j < numEnds[i].y; ++j)
        {
            FilamentEndState state = (FilamentEndState)(endStates[kdx] & 0x7FFF);
            bool             forbid = (bool)(endStates[kdx] & 0x8000);

            stack->mSectionInterfaces[i].mMatching.setState((FilamentEndState)endStates[kdx], j / 2, j % 2, 1);

            if (stack->mSectionInterfaces[i].mMatching.isLandmarkForbidden(j / 2, j % 2, 1) != forbid)
            {
                stack->mSectionInterfaces[i].mMatching.switchForbiddenState(j / 2, j % 2, 1);
            }

            kdx++;
        }

        // matching

        stack->mSectionInterfaces[i].mMatching.clear();

        for (int j = 0; j < numMatching[i]; ++j)
        {
            stack->mSectionInterfaces[i].mMatching.add(matchings[jdx * 2], matchings[jdx * 2 + 1], matchingColors[jdx], (SettingType)matchingTypes[jdx]);
            jdx++;
        }
    }

    stack->setLoadCmd(QString::fromLatin1(filename));
    stack->setSaveInfoParam("AmiraMesh");
    stack->updateInfo();
    stack->fire();

    HxData::registerData(stack, QString::fromLatin1(filename));

    return 1;
}




void HxSerialSectionStack::removeSection(int sectionID)
{
    mSections.remove(sectionID);

    if (mSectionInterfaces.size() > 0)
    {
        if (sectionID < mSectionInterfaces.size())
        {
            mSectionInterfaces.remove(sectionID);
        }
        else
        {
            mSectionInterfaces.remove(sectionID - 1);
        }
    }

    touch();
    touchConnections();
    updateInfo();
}




int HxSerialSectionStack::saveAmiraMesh(const char* filename)
{
    AmiraMesh m;
    m.parameters.copy(HxParameter("ContentType","HxSerialSectionStack"));

    HxParamBundle* rootBundle = new HxParamBundle("Sections");
    m.parameters.insert(rootBundle, 0);

    for (int i=0; i<getNumSections(); ++i) {
        McString sectionName = McString().printf("Section%04d", i);
        HxParamBundle* sectionBundle = new HxParamBundle(QString::fromLatin1(sectionName.getString()));

        QString reconstructionFN("");

        if (mSections[i].mFilaments)
        {
            reconstructionFN = mSections[i].mFilaments->getFilename();
        }

        QString   fileName = QString::fromLatin1(filename);
        QFileInfo file(fileName);
        QDir      filePath(file.path());
        QString   relImage = filePath.relativeFilePath(mSections[i].mImageFileName);
        QString   relReconstruction = filePath.relativeFilePath(reconstructionFN);

        if (relImage != "")
        {
            sectionBundle->set("Image", relImage);
        }
        if (relReconstruction != "")
        {
            sectionBundle->set("Reconstruction", relReconstruction);
        }

        rootBundle->insert(sectionBundle, 0);
    }

    // sections

    int numSections = mSections.size();

    McDArray< float > imageBoxes(numSections * 6);
    McDArray< int >   imageDimensions(numSections * 3);

    for (int i = 0; i < numSections; ++i)
    {
        imageBoxes[i * 6 + 0] = mSections[i].mImageBoundingBox[0];
        imageBoxes[i * 6 + 1] = mSections[i].mImageBoundingBox[1];
        imageBoxes[i * 6 + 2] = mSections[i].mImageBoundingBox[2];
        imageBoxes[i * 6 + 3] = mSections[i].mImageBoundingBox[3];
        imageBoxes[i * 6 + 4] = mSections[i].mImageBoundingBox[4];
        imageBoxes[i * 6 + 5] = mSections[i].mImageBoundingBox[5];
        imageDimensions[i * 3 + 0] = mSections[i].mImageDimensions[0];
        imageDimensions[i * 3 + 1] = mSections[i].mImageDimensions[1];
        imageDimensions[i * 3 + 2] = mSections[i].mImageDimensions[2];
    }

    // section interfaces

    int numInterfaces     = mSectionInterfaces.size();
    int numLandmarks      = 0;
    int numMatchings      = 0;
    int numOverallEnds    = 0;

    McDArray< int >     numMarks(numInterfaces);
    McDArray< int >     numMatching(numInterfaces);
    McDArray< McVec2i > numFilamentEnds(numInterfaces);


    for (int i = 0; i < numInterfaces; ++i)
    {
        numMarks[i]        = mSectionInterfaces[i].mLandmarks.getNumMarks();
        numMatching[i]     = mSectionInterfaces[i].mMatching.getNumMatches();
        numFilamentEnds[i] = McVec2i(mSectionInterfaces[i].mMatching.getNumStates(0),
                                     mSectionInterfaces[i].mMatching.getNumStates(1));

        numLandmarks   += numMarks[i];
        numMatchings   += numMatching[i];
        numOverallEnds += numFilamentEnds[i].x + numFilamentEnds[i].y;
    }

    McDArray< McVec2f > landmarks(0, numLandmarks * 2);
    McDArray< int >     landmarkTypes(0, numLandmarks);
    McDArray< McVec4i > matchings(0, numMatchings);
    McDArray< McVec3f > matchingColors(0, numMatchings);
    McDArray< int >     matchingTypes(0, numMatchings);
    McDArray< int >     endStates(0, numOverallEnds);

    for (int i = 0;  i < numInterfaces; ++i)
    {
        for (int j = 0; j < numMarks[i]; ++j)
        {
            landmarks.push(mSectionInterfaces[i].mLandmarks.getMark(j, 0));
            landmarks.push(mSectionInterfaces[i].mLandmarks.getMark(j, 1));
            landmarkTypes.push(mSectionInterfaces[i].mLandmarks.getType(j));
        }

        for (int j = 0; j < numMatching[i]; ++j)
        {
            matchings.push(mSectionInterfaces[i].mMatching.get(j));
            matchingColors.push(mSectionInterfaces[i].mMatching.getColor(j));
            matchingTypes.push(mSectionInterfaces[i].mMatching.getType(j));
        }

        for (int j = 0; j < mSectionInterfaces[i].mMatching.getNumStates(0); ++j)
        {
            int f = mSectionInterfaces[i].mMatching.isLandmarkForbidden(j / 2, j % 2, 0) ? 1 : 0;

            endStates.push(mSectionInterfaces[i].mMatching.getState(j / 2, j % 2, 0));
            endStates[endStates.size() - 1] |= f << 15;
        }
        for (int j = 0; j < mSectionInterfaces[i].mMatching.getNumStates(1); ++j)
        {
            int f = mSectionInterfaces[i].mMatching.isLandmarkForbidden(j / 2, j % 2, 1) ? 1 : 0;

            endStates.push(mSectionInterfaces[i].mMatching.getState(j / 2, j % 2, 1));
            endStates[endStates.size() - 1] |= f << 15;
        }
    }

    // build amira mesh

    AmiraMesh::Location* sectionLocation          = new AmiraMesh::Location("SECTIONS", numSections);
    AmiraMesh::Location* sectionInterfaceLocation = new AmiraMesh::Location("SECTION_CHANGES", numInterfaces);
    AmiraMesh::Location* landmarkLocation         = new AmiraMesh::Location("LANDMARKS", numLandmarks);
    AmiraMesh::Location* matchingLocation         = new AmiraMesh::Location("MATCHINGS", numMatchings);
    AmiraMesh::Location* endLocation              = new AmiraMesh::Location("MICROTUBULE_ENDS", numOverallEnds);

    m.locationList.append(sectionLocation);
    m.locationList.append(sectionInterfaceLocation);
    m.locationList.append(landmarkLocation);
    m.locationList.append(matchingLocation);
    m.locationList.append(endLocation);

    AmiraMesh::Data* sectionImageBoxes = new AmiraMesh::Data("ImageBoxes",      sectionLocation,          McPrimType::MC_FLOAT, 6, imageBoxes.dataPtr());
    AmiraMesh::Data* sectionImageDims  = new AmiraMesh::Data("ImageDimensions", sectionLocation,          McPrimType::MC_INT32, 3, imageDimensions.dataPtr());
    AmiraMesh::Data* numLandmarksData  = new AmiraMesh::Data("Landmarks",       sectionInterfaceLocation, McPrimType::MC_INT32, 1, numMarks.dataPtr());
    AmiraMesh::Data* numMatchingsData  = new AmiraMesh::Data("Matchings",       sectionInterfaceLocation, McPrimType::MC_INT32, 1, numMatching.dataPtr());
    AmiraMesh::Data* numEndsData       = new AmiraMesh::Data("MicrotubuleEnds", sectionInterfaceLocation, McPrimType::MC_INT32, 2, numFilamentEnds.dataPtr());
    AmiraMesh::Data* landmarkData      = new AmiraMesh::Data("Coordinates",     landmarkLocation,         McPrimType::MC_FLOAT, 4, landmarks.dataPtr());
    AmiraMesh::Data* landmarkTypesData = new AmiraMesh::Data("Types",           landmarkLocation,         McPrimType::MC_INT32, 1, landmarkTypes.dataPtr());
    AmiraMesh::Data* matchingData      = new AmiraMesh::Data("Matches",         matchingLocation,         McPrimType::MC_INT32, 4, matchings.dataPtr());
    AmiraMesh::Data* matchingColorData = new AmiraMesh::Data("Colors",          matchingLocation,         McPrimType::MC_FLOAT, 3, matchingColors.dataPtr());
    AmiraMesh::Data* matchingTypesData = new AmiraMesh::Data("Types",           matchingLocation,         McPrimType::MC_INT32, 1, matchingTypes.dataPtr());
    AmiraMesh::Data* endStatesData     = new AmiraMesh::Data("States",          endLocation,              McPrimType::MC_INT32, 1, endStates.dataPtr());

    m.dataList.append(sectionImageBoxes);
    m.dataList.append(sectionImageDims);
    m.dataList.append(numLandmarksData);
    m.dataList.append(numMatchingsData);
    m.dataList.append(numEndsData);
    m.dataList.append(landmarkData);
    m.dataList.append(landmarkTypesData);
    m.dataList.append(matchingData);
    m.dataList.append(matchingColorData);
    m.dataList.append(matchingTypesData);
    m.dataList.append(endStatesData);

    m.write(filename, AmiraMesh::AM_ASCII);

    setLoadCmd(QString::fromLatin1(filename));
    setModified(0);

    return 1;
}




void HxSerialSectionStack::setFilaments(McHandle<HxSpatialGraph> graph, const int sectionID)
{
    mcassert(graph);
    mcassert(sectionID < mSections.size());

    int sectionNumber = -1;

    if (sectionID >= 0)
    {
        sectionNumber = sectionID;
    }
    else
    {
        // Insert at empty position
        bool emptyPositionFound = false;
        for (int i = 0; i < mSections.size(); ++i)
        {
            if (mSections[i].mFilaments == 0)
            {
                sectionNumber = i;
                emptyPositionFound = true;
                break;
            }
        }

        // Or else add new section
        if (!emptyPositionFound)
        {
            sectionNumber = addSection();
        }
    }

    mSections[sectionNumber].mFilaments = graph;
}




void HxSerialSectionStack::setImage(McHandle<HxUniformScalarField3> image, const int sectionID)
{
    mcassert(image);
    mcassert(sectionID < mSections.size());

    int sectionNumber = -1;

    if (sectionID >= 0)
    {
        sectionNumber = sectionID;
    }
    else
    {
        // Insert at empty position
        bool emptyPositionFound = false;

        for (int i = 0; i < mSections.size(); ++i)
        {
            if (mSections[i].mImageFileName == "")
            {
                sectionNumber = i;
                emptyPositionFound = true;
                break;
            }
        }

        // Or else add new section
        if (!emptyPositionFound)
        {
            sectionNumber = addSection();
        }
    }

    mSections[sectionNumber].mImageFileName = image->getFilename();
    mSections[sectionNumber].mImageDimensions = image->lattice().getDims();
    mSections[sectionNumber].mImageBoundingBox = image->getBoundingBox();
}




void HxSerialSectionStack::touchConnections()
{
    int n = downStreamConnections.size();

    for (int i = 0; i < n; ++i)
    {
        downStreamConnections[i]->touch(HxData::NEW_SOURCE);
    }
}




void HxSerialSectionStack::update ()
{
}




void HxSerialSectionStack::updateInfo()
{
    int n = getNumSections();

    QString infoText;

    for (int i = 0; i < n; ++i)
    {
        QString   fileName = mSections[i].mImageFileName;
        QFileInfo file(fileName);
        QString   fileNameExt = file.fileName();

        infoText += QString::number(i) + QString(": ") + fileNameExt + QString("\n");
    }

    portInfo.printf("%s", infoText.toUtf8().constData());
}

