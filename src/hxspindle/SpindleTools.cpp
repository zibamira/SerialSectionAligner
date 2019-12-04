#include <hxalignmicrotubules/MovingLeastSquares.h>
#include <hxcore/HxCompModule.h>
#include <hxcore/HxPortMultiMenu.h>
#include <hxcore/HxPort3DPointList.h>
#include <hxfield/HxUniformScalarField3.h>
#include <hxspindle/SpindleTools.h>
#include <mclib/McVec2.h>
#include <mclib/McVec4.h>




float SpindleTools::closestEdgePoint(const McVec3f&        point,
                                     const HxSpatialGraph& graph,
                                     const int             edgeID,
                                           int&            segmentID,
                                           McVec3f&        closestPoint)
{
    float minDistance = FLT_MAX;

    const int numPoints = graph.getNumEdgePoints(edgeID);

    for (int i = 1; i < numPoints; ++i)
    {
        const McVec3f segStart = graph.getEdgePoint(edgeID, i - 1);
        const McVec3f segEnd   = graph.getEdgePoint(edgeID, i);
        const McVec3f segPoint = closestLinePoint(point, segStart, segEnd);

        const float distance = (segPoint - point).length();

        if (distance < minDistance)
        {
            minDistance = distance;
            closestPoint = segPoint;
            segmentID = i - 1;
        }
    }

    return minDistance;
}




float SpindleTools::closestEdgePoint(const McVec3f&             point,
                                     const McDArray< McVec3f >& edge,
                                           int&                 segmentID,
                                           McVec3f&             closestPoint)
{
    float minDistance = FLT_MAX;

    const int numPoints = edge.size();

    for (int i = 1; i < numPoints; ++i)
    {
        const McVec3f segStart = edge[i - 1];
        const McVec3f segEnd = edge[i];
        const McVec3f segPoint = closestLinePoint(point, segStart, segEnd);

        const float distance = (segPoint - point).length();

        if (distance < minDistance)
        {
            minDistance = distance;
            closestPoint = segPoint;
            segmentID = i - 1;
        }
    }

    return minDistance;
}




McVec3f SpindleTools::closestLinePoint(const McVec3f& point,
                                       const McVec3f& lineStart,
                                       const McVec3f& lineEnd)
{
    McVec3f rS = lineStart;
    McVec3f rD = lineEnd - lineStart;
    McVec3f p = point;

    double t = rD.dot(p - rS) / rD.dot(rD);

    if (t < 0.0)
    {
        return lineStart;
    }
    else if (t > 1.0)
    {
        return lineEnd;
    }
    else
    {
        return rS + t * rD;
    }
}




double SpindleTools::computeAngle(const HxSpatialGraph& microtubules, int edge1, int edge2)
{
    int n1 = microtubules.getNumEdgePoints(edge1);
    int n2 = microtubules.getNumEdgePoints(edge2);

    if (n1 < 2 || n2 < 2) return 0.0;

    McVec3f v1 = microtubules.getEdgePoint(edge1, 0) - microtubules.getEdgePoint(edge1, n1 - 1);
    McVec3f v2 = microtubules.getEdgePoint(edge2, 0) - microtubules.getEdgePoint(edge2, n2 - 1);

    if (v1.dot(v2) < 0.0) v1 = -v1;

    return (v1.angle(v2) / M_PI) * 180.0;
}




double SpindleTools::computeDistance(const HxSpatialGraph& microtubules, int edge1, int edge2)
{
    int numPoints1 = microtubules.getNumEdgePoints(edge1);
    int numPoints2 = microtubules.getNumEdgePoints(edge2);

    double minDist = DBL_MAX;

    if (numPoints1 < 2 || numPoints2 < 2) return minDist;

    for (int i = 1; i < numPoints1; ++i)
    {
        for (int j = 1; j < numPoints2; ++j)
        {
            double d = lineDistance(microtubules.getEdgePoint(edge1, i - 1), microtubules.getEdgePoint(edge1, i),
                                    microtubules.getEdgePoint(edge2, j - 1), microtubules.getEdgePoint(edge2, j));

            if (d < minDist) minDist = d;
        }
    }

    return minDist;
}




float SpindleTools::computeIntersectionArea(const HxUniformScalarField3& field1, const HxUniformScalarField3& field2, const MovingLeastSquares& mls, int resolutionDivisor)
{
    McDim3l dims = field1.lattice().getDims();
    McBox3f box  = field2.getBoundingBox();

    int   voxels     = 0;
    float voxelArea  = field1.getVoxelSize().x * field1.getVoxelSize().y;
          voxelArea *= float(resolutionDivisor * resolutionDivisor);

    #ifdef _OPENMP
        #pragma omp parallel for
    #endif

    for (int x = 0; x < dims.nx; x += resolutionDivisor)
    {
        for (int y = 0; y < dims.ny; y += resolutionDivisor)
        {
            McVec3f p = field1.lattice().coords()->pos(McVec3i(x, y, 0));
            McVec2d q = McVec2d(p.x, p.y);
            McVec2d qInv = mls.interpolateFast(q, 9);

            if (qInv.x < box.getMin().x || qInv.x > box.getMax().x ||
                qInv.y < box.getMin().y || qInv.y > box.getMax().y)
            {
                continue;
            }

            #ifdef _OPENMP
                #pragma omp atomic
            #endif

            voxels++;
        }
    }

    return voxels * voxelArea;
}




float SpindleTools::computeLength(const McDArray<McVec3f>& points)
{
    float l = 0.0f;

    for (int i = 1; i < points.size(); ++i)
    {
        l += (points[i] - points[i - 1]).length();
    }

    return l;
}




bool SpindleTools::computePlaneIsec(const McDArray< McVec3f >& points, const McVec4f& plane, McVec3f& point)
{
    float   d = plane.t;
    McVec3f n = McVec3f(plane.x, plane.y, plane.z);

    for (int i = 1; i < points.size(); ++i)
    {
        McVec3f p1 = points[i - 1];
        McVec3f p2 = points[i];

        float d1 = n.dot(p1) - d;
        float d2 = n.dot(p2) - d;

        if (d1 == 0.0f)
        {
            point = p1;
            return true;
        }

        if (d2 == 0.0f)
        {
            point = p2;
            return true;
        }

        if (d1 * d2 > 0.0) continue;

        float t = (d - n.dot(p1)) / n.dot(p2 - p1);

        point = p1 + t * (p2 - p1);

        return true;
    }

    return false;
}




McVec3f SpindleTools::computePointAtDist(const McDArray<McVec3f>& points, const float maxDist, int startIdx, int increment)
{
    float dist = 0.0;
    int   idx  = startIdx;
    int   n    = points.size();

    while (dist < maxDist && idx < n && idx >= 0)
    {
        dist += (points[idx - increment] - points[idx]).length();
        idx  += increment;
    }

    // max distance was not reached at all.

    if (dist <= maxDist)
    {
        return points[idx - increment];
    }

    // interpolate the last point.

    else
    {
        const McVec3f& x2 = points[ idx - increment];
        const McVec3f& x1 = points[(idx - increment) - increment];

        const float diffDist       = dist - maxDist;
        const float distLastPoints = (x2 - x1).length();
            
        float frac = diffDist / distLastPoints;

        return (1 - frac) * x2 + frac * x1;
    }
}




McVec3f SpindleTools::computePointAtDist(const McDArray<McVec3f>& points, const float maxDist, int startIdx, int increment, float& errorDist)
{
    McDArray< McVec3f > usedPoints(0, 100);

    float dist = 0.0;
    int   idx  = startIdx;
    int   n    = points.size();

    usedPoints.push_back(points[idx - increment]);

    while (dist < maxDist && idx < n && idx >= 0)
    {
        usedPoints.push_back(points[idx]);

        dist += (points[idx - increment] - points[idx]).length();
        idx  += increment;
    }

    // interpolate the last point.

    if (dist > maxDist)
    {
        const McVec3f& x2 = points[ idx - increment];
        const McVec3f& x1 = points[(idx - increment) - increment];

        const float diffDist       = dist - maxDist;
        const float distLastPoints = (x2 - x1).length();
            
        float frac = diffDist / distLastPoints;

        usedPoints.push_back((1 - frac) * x2 + frac * x1);
    }

    // error analysis.

    errorDist = 0.0;

    McVec3f p = usedPoints[0];

    for (int i = 1; i < usedPoints.size(); ++i)
    {
        McVec3f q = usedPoints[i];

        for (int j = 1; j < i; ++j)
        {
            McVec3f k = closestLinePoint(p, q, usedPoints[j]);
            float   e = (k - usedPoints[j]).length() / (p - q).length();

            errorDist = std::max(errorDist, e);
        }
    }

    return usedPoints[usedPoints.size() - 1];
}





McVec3f SpindleTools::computeTangent(const McDArray<McVec3f>& points, int pointID)
{
    int nextID = std::min(pointID + 1, (int) points.size() - 1);
    int prevID = std::max(pointID - 1, (int) 0);

    McVec3f t = points[nextID] - points[prevID];

    t.normalize();

    return t;
}




HxSpatialGraph* SpindleTools::createOrReuse(HxCompModule* module, const QString& label, const QString extension, int resultID)
{
    HxSpatialGraph* graph = dynamic_cast< HxSpatialGraph* >(module->getResult(resultID));

    if (graph && graph->isOfType(HxSpatialGraph::getClassTypeId()))
    {
        graph->clear();
    }
    else
    {
        graph = HxSpatialGraph::createInstance();
    }

    graph->composeLabel(label, extension);

    return graph;
}




McDArray< McVec3f > SpindleTools::getCentrosomes(const HxSpatialGraph& graph)
{
    McDArray< McVec3f > centrosomes(0);

    const HxParamBundle* centrosomesBundle = graph.parameters.getBundle("Centrosomes");

    if (!centrosomesBundle)
    {
        return centrosomes;
    }

    int numCentrosomes = centrosomesBundle->getNumberOfBundles();


    for (int i = 0; i < numCentrosomes; ++i)
    {
        const HxParamBundle* centrosomeBundle = centrosomesBundle->getBundle(i);

        double x, y, z;

        if (!centrosomeBundle->findReal("X", x)) continue;
        if (!centrosomeBundle->findReal("Y", y)) continue;
        if (!centrosomeBundle->findReal("Z", z)) continue;

        centrosomes.push(McVec3f((float) x, (float) y, (float) z));
    }

    return centrosomes;
}




double SpindleTools::lineDistance(const McVec3f& lineStart1,
                                  const McVec3f& lineEnd1,
                                  const McVec3f& lineStart2,
                                  const McVec3f& lineEnd2)
{
    McVec3f d1 = lineEnd1   - lineStart1;
    McVec3f d2 = lineEnd2   - lineStart2;
    McVec3f s  = lineStart1 - lineStart2;

    /*
        We will compute "t1" and "t2" for which a vector "n"
        connecting the two lines with the closest distance.

        n = lineStart1 + t1 * d1 - lineStart2 - t2 + d2
        n = s + t1 * d1 - t2 * d2

        Note, that "n" is orthogonal to "d1" and "d2"

        n * d1 = 0 = d1 * s + t1 * d1 * d1 - t2 * d2 * d1
        n * d2 = 0 = d2 * s + t1 * d1 * d2 - t2 * d2 * d2

        We have to solve this system to get "t1" and "t2"
    */

    double q1 = d1.dot(s);
    double q2 = d2.dot(s);

    double d1s = d1.dot(d1);
    double d2s = d2.dot(d2);
    double d12 = d1.dot(d2);

    /*
        The system becomes now

        t1 * d1s - t2 * d12 = -q1
        t1 * d12 - t2 * d2s = -q2   =>   t1 = (t2 * d2s -q2) / d12

        => (t2 * d2s -q2) / d12 * d1s - t2 * d12 = -q1

        => t2 = (q2 * d1s - q1 * d12) / (d2s * d1s - d12 * d12)
        => t1 = (q2 * d12 - q1 * d2s) / (d2s * d1s - d12 * d12)
    */

    double w = d2s * d1s - d12 * d12;

    /*
        If "w = 0" (division by zero), the lines are parallel and
        there exist an infinite number of vectors "n", so we can
        fix "t1" or "t2" by 0, thus for "t1 = 0".

        t2 = q1 / d12 or t2 = q2 / d2s
    */

    double t1D = w;        // denominator for t1
    double t1N = 0.0;      // numerator for t1
    double t2D = w;        // denominator for t2
    double t2N = 0.0;      // numerator for t2


    if (w <= 0.0)
    {
        t1N = 0.0;
        t1D = 1.0;
        t2N = q2;
        t2D = d2s;
    }
    else
    {
        t1N = q2 * d12 - q1 * d2s;
        t2N = q2 * d1s - q1 * d12;

        // check ranges for t1

        if (t1N < 0.0)
        {
            t1N = 0.0;
            t2N = q2;
            t2D = d2s;
        }
        else if (t1N > t1D)
        {
            t1N = t1D;
            t2N = q2 + d12;
            t2D = d2s;
        }
    }

    // check ranges for t2

    if (t2N < 0.0)
    {
        t2N = 0.0;

        if      (-q1 < 0.0) t1N = 0.0;
        else if (-q1 > d1s) t1N = t1D;
        else
        {
            t1N = -q1;
            t1D = d1s;
        }
    }
    else if (t2N > t2D)
    {
        t2N = t2D;

        if      ((-q1 + d12) < 0.0) t1N = 0.0;
        else if ((-q1 + d12) > d1s) t1N = t1D;
        else
        {
            t1N = d12 - q1;
            t1D = d1s;
        }
    }

    // get the difference of the two closest points

    double t1 = (abs(t1N) <= 0.0 ? 0.0 : t1N / t1D);
    double t2 = (abs(t2N) <= 0.0 ? 0.0 : t2N / t2D);

    McVec3f n = s + (t1 * d1) - (t2 * d2);

    return n.length();
}




float SpindleTools::pointEdgeDistance(const McVec3f&        point,
                                           const HxSpatialGraph& microtubules,
                                           const int             microtubuleID)
{
    float minDist = FLT_MAX;
    int   numPoints = microtubules.getNumEdgePoints(microtubuleID);

    for (int i = 1; i < numPoints; ++i)
    {
        McVec3f segStart = microtubules.getEdgePoint(microtubuleID, i - 1);
        McVec3f segEnd   = microtubules.getEdgePoint(microtubuleID, i);
        McVec3f segPoint = closestLinePoint(point, segStart, segEnd);

        minDist = std::min(minDist, (point - segPoint).length());
    }

    return minDist;
}





void SpindleTools::setEdgeAtt(const HxSpatialGraph* graph, HxPortMultiMenu& multiMenu)
{
    if (!graph)
    {
        multiMenu.setNum(0, 0);
        return;
    }

    int numEdgeAttributes = graph->numAttributes(HxSpatialGraph::EDGE);
    int numEdgeAttributesUsed = 0;

    for (int i = 0; i < numEdgeAttributes; ++i)
    {
        GraphAttribute* att = graph->attribute(HxSpatialGraph::EDGE, i);

        if (att->primType() != McPrimType::MC_INT32 &&
            att->primType() != McPrimType::MC_FLOAT) continue;

        numEdgeAttributesUsed++;
    }

    multiMenu.setNum(0, numEdgeAttributesUsed);

    int attIdx = 0;

    for (int i = 0; i < numEdgeAttributes; ++i)
    {
        GraphAttribute* att = graph->attribute(HxSpatialGraph::EDGE, i);

        if (att->primType() != McPrimType::MC_INT32 &&
            att->primType() != McPrimType::MC_FLOAT) continue;

        multiMenu.setLabel(0, attIdx, QString::fromLatin1(att->getName()));
        attIdx++;
    }
}




void SpindleTools::setEdgeIntAtt(const HxSpatialGraph* graph, HxPortMultiMenu& multiMenu)
{
    if (!graph)
    {
        multiMenu.setNum(0, 0);
        return;
    }

    int numEdgeAttributes = graph->numAttributes(HxSpatialGraph::EDGE);
    int numEdgeAttributesInt = 0;

    for (int i = 0; i < numEdgeAttributes; ++i)
    {
        GraphAttribute* att = graph->attribute(HxSpatialGraph::EDGE, i);

        if (att->primType() != McPrimType::MC_INT32) continue;

        numEdgeAttributesInt++;
    }

    multiMenu.setNum(0, numEdgeAttributesInt);

    int attIdx = 0;

    for (int i = 0; i < numEdgeAttributes; ++i)
    {
        GraphAttribute* att = graph->attribute(HxSpatialGraph::EDGE, i);

        if (att->primType() != McPrimType::MC_INT32) continue;

        multiMenu.setLabel(0, attIdx, QString::fromLatin1(att->getName()));
        attIdx++;
    }
}




void SpindleTools::setEdgePointAtt(const HxSpatialGraph* graph, HxPortMultiMenu& multiMenu)
{
    if (!graph)
    {
        multiMenu.setNum(0, 0);
        return;
    }

    int numEdgeAttributes = graph->numAttributes(HxSpatialGraph::EDGE);
    int numPointAttributes = graph->numAttributes(HxSpatialGraph::POINT);
    int numAttributesUsed = 0;

    for (int i = 0; i < numEdgeAttributes; ++i)
    {
        GraphAttribute* att = graph->attribute(HxSpatialGraph::EDGE, i);

        if (att->primType() != McPrimType::MC_INT32 &&
            att->primType() != McPrimType::MC_FLOAT) continue;

        numAttributesUsed++;
    }

    for (int i = 0; i < numPointAttributes; ++i)
    {
        GraphAttribute* att = graph->attribute(HxSpatialGraph::POINT, i);

        if (att->primType() != McPrimType::MC_INT32 &&
            att->primType() != McPrimType::MC_FLOAT) continue;

        numAttributesUsed++;
    }

    multiMenu.setNum(0, numAttributesUsed);

    int attIdx = 0;

    for (int i = 0; i < numEdgeAttributes; ++i)
    {
        GraphAttribute* att = graph->attribute(HxSpatialGraph::EDGE, i);

        if (att->primType() != McPrimType::MC_INT32 &&
            att->primType() != McPrimType::MC_FLOAT) continue;

        multiMenu.setLabel(0, attIdx, QString("edge: ") + QString::fromLatin1(att->getName()));
        attIdx++;
    }

    for (int i = 0; i < numPointAttributes; ++i)
    {
        GraphAttribute* att = graph->attribute(HxSpatialGraph::POINT, i);

        if (att->primType() != McPrimType::MC_INT32 &&
            att->primType() != McPrimType::MC_FLOAT) continue;

        multiMenu.setLabel(0, attIdx, QString("point: ") + QString::fromLatin1(att->getName()));
        attIdx++;
    }
}




void SpindleTools::setPoints(const HxSpatialGraph* graph, McHandle<HxPort3DPointList> points)
{
    if (graph)
    {
        McDArray< McVec3f > centrosomes = SpindleTools::getCentrosomes(*graph);

        if (centrosomes.size() < 1)
        {
            points->setNumPoints(1);
            points->setCoord(0, SbVec3d(0.0, 0.0, 0.0));
        }
        else
        {
            points->setNumPoints(centrosomes.size());

            for (int i = 0; i < centrosomes.size(); ++i)
            {
                points->setCoord(i, SbVec3d(centrosomes[i].x, centrosomes[i].y, centrosomes[i].z));
            }
        }
    }
    else
    {
        points->setNumPoints(1);
        points->setCoord(0, SbVec3d(0.0, 0.0, 0.0));
    }
}




McVec2f SpindleTools::vec2(const McVec3f& point)
{
    return McVec2f(point.x, point.y);
}

