#pragma once

#include "hxspindle/api.h"
#include <hxspatialgraph/internal/HxSpatialGraph.h>
#include <mclib/McVec2.h>
#include <mclib/McVec3.h>
#include <mclib/McVec4.h>

class HxPort3DPointList;
class HxPortMultiMenu;
class HxUniformScalarField3;
class MovingLeastSquares;




class HXSPINDLE_API SpindleTools
{
    public:

        /**
            Computes the point on a spatial graph edge, which is closest
            to a user-defined point. The method returns the distance as
            well as the point and the segment id of the edge.
        */
        static float closestEdgePoint(const McVec3f&        point,
                                      const HxSpatialGraph& graph,
                                      const int             edgeID,
                                            int&            segmentID,
                                            McVec3f&        closestPoint);


        /**
            Computes the point on a spatial graph edge, which is closest
            to a user-defined point. The method returns the distance as
            well as the point and the segment id of the edge.
        */
        static float closestEdgePoint(const McVec3f&             point,
                                      const McDArray< McVec3f >& edge,
                                            int&                 segmentID,
                                            McVec3f&             closestPoint);


        /**
            Computes the point on a line segment with the
            minimal distance to user-defined point.
        */
        static McVec3f closestLinePoint(const McVec3f& point,
                                        const McVec3f& lineStart,
                                        const McVec3f& lineEnd);


        /**
            Computes the angle between two edges. The angel is given
            by the angle between the lines that connect the start and
            end point of the microtubules.
        */
        static double computeAngle(const HxSpatialGraph& microtubules, int edge1, int edge2);


        /**
            Computes the minimal distance between two edges
            of the spatial graph.
        */
        static double computeDistance(const HxSpatialGraph& microtubules, int edge1, int edge2);


        /**
            Computes the intersection area of two sections. To do so, each
            grid point of field 1 is transformed to field 2 using moving least
            squares. If the transformed grid point lies inside the bounding box
            of field 2 the area of the lattice rectangle is add to the overall
            area. To accelerate, the user can reduce the resolution, which also
            reduces the accuracy of the result.
        */
        static float computeIntersectionArea(const HxUniformScalarField3& field1, const HxUniformScalarField3& field2, const MovingLeastSquares& mls, int resolutionDivisor);


        /**
            Computes the length of a poly-line.
        */
        static float computeLength(const McDArray< McVec3f >& points);


        /**
            Computes the intersection of a poly-line and a plane.
            Returns false if there is no intersection.
        */
        static bool computePlaneIsec(const McDArray< McVec3f >& points, const McVec4f& plane, McVec3f& point);


        /**
            Given a poly-line as list of points, a point on the line is
            computed that has a maximal distance "maxDist" to the start
            point given by the index of the point in the list. The
            direction is given by increment and shall be either 1 or
            -1. If the poly-line is shorter than "maxDist", depending
            on the direction, the endpoint of the list is returned.
        */
        static McVec3f computePointAtDist(const McDArray<McVec3f>& points, const float maxDist, int startIdx, int increment);


        /**
            Given a poly-line as list of points, a point on the line is
            computed that has a maximal distance "maxDist" to the start
            point given by the index of the point in the list. The
            direction is given by increment and shall be either 1 or
            -1. If the poly-line is shorter than "maxDist", depending
            on the direction, the endpoint of the list is returned.
        */
        static McVec3f computePointAtDist(const McDArray<McVec3f>& points, const float maxDist, int startIdx, int increment, float& errorDist);


        /**
            Computes the tangent at a position of a poly-line.
        */
        static McVec3f computeTangent(const McDArray< McVec3f >& points, int pointID);


        /**
            Detects if the module has already a spatial graph in this result
            slot. In this case the result name is updated and the graph is
            cleared and returned. Otherwise a new empty graph is created.
        */
        static HxSpatialGraph* createOrReuse(HxCompModule* module, const QString& label, const QString extension, int resultID = 0);


        /**
            Returns the centrosomes from the parameters in the spatial graph.
        */
        static McDArray< McVec3f > getCentrosomes(const HxSpatialGraph& graph);

        /**
            Computes the minimal distance between two line
            segments.
        */
        static double lineDistance(const McVec3f& lineStart1,
                                   const McVec3f& lineEnd1,
                                   const McVec3f& lineStart2,
                                   const McVec3f& lineEnd2);


        /**
            Computes the minimal distance of a spatial graph edge
            (microtubule) to an arbitrary point.
        */
        static float pointEdgeDistance(const McVec3f&        point,
                                       const HxSpatialGraph& microtubules,
                                       const int             microtubuleID);


        /**
            Sets all labels of the input microtubules in the portData
            to the selected multi menu.
        */
        static void setEdgeAtt(const HxSpatialGraph* graph, HxPortMultiMenu& multiMenu);


        /**
            Sets all labels of the input microtubules in the portData
            to the selected multi menu.
        */
        static void setEdgeIntAtt(const HxSpatialGraph* graph, HxPortMultiMenu& multiMenu);


        /**
            Puts all edge and point attributes of the input microtubules
            in to the selected multi menu.
        */
        static void setEdgePointAtt(const HxSpatialGraph* graph, HxPortMultiMenu& multiMenu);


        /**
            Sets the centrosome points from the parameters of the graph
            to the port points. If the parameter do not exist only a single
            point is created with coordinates 0.
        */
        static void setPoints(const HxSpatialGraph* graph, McHandle< HxPort3DPointList > points);


        /**
            Returns the x,y coordinates of a 3D point as 2D point.
        */
        static McVec2f vec2(const McVec3f& point);

};


