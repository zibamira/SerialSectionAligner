#ifndef MC_DIJKSTRA_SHORTEST_PATH_H
#define MC_DIJKSTRA_SHORTEST_PATH_H

#include <mclib/McDArray.h>
#include <mclib/internal/McFHeap.h>
#include <mclib/internal/McList.h>

#include "api.h"

/** 
    The class @c McDijkstraEdge implements a directed edge. It has
    three member variables: two indices of the start and end vertices,
    and the edge length which is of type @c float. Note, that the edge
    length needs to be non-negative for the Dijkstra algorithm to
    work.
*/
class HXGRAPHALGORITHMS_API McDijkstraEdge : public McLink {
public:
    int startVertex;
    int endVertex;
    double edgeLength;
	int id;
    
    /// Comparison operator.
    int operator<(const McDijkstraEdge & edge) const
    { return (edgeLength < edge.edgeLength); }
    /// Comparison operator.
    int operator>(const McDijkstraEdge & edge) const
    { return (edgeLength > edge.edgeLength); }
};

/** 
    The class $c McDijkstraVertex implements a node of a directed
    graph. It contains a couple of member variables:
    - @c idx is the index of the vertex
    - @c distance is the current shortest distance from some specified vertex.
    - @c predEdge is a pointer to the last edge of the current shortest 
    path to this vertex
    - @c edges is the list of outgoing edges
    Only the @c edges list needs to be initialized before calling the
    Dijkstra algorithm. All other member variables will be initialized
    by the algorithm itself.
    
    The class also provides a smaller-than operator which is needed for
    the vertex as heap element.
*/
class HXGRAPHALGORITHMS_API McDijkstraVertex : public McFHeapElement {
public:
    int idx;
    double distance;
    McDijkstraEdge * predEdge;
    McList<McDijkstraEdge> edges;
    
    /// Comparison operator needed for heap.
    int operator<(const McDijkstraVertex & vertex)
    { return (distance < vertex.distance); }
};

/** 
    The class @c McDijkstraShortestPath provides two static functions
    to compute the shortest path between two specified vertices, and
    the shortest paths between a single specified vertex and all other
    vertices, respectively. Note that, since we use pointers to the
    edges in our implementation, some care needs to be taken when
    creating the edges. You <em> must not </em> resize (or append an
    item to) the array after adding edges to the array, i.e., after
    assiging any pointers to elements in the array.
    
    The shortest path(s) is (are) only given implicetely and can be
    computed by backtracking from the end vertex using the @c predEdge
    pointer. The @c startVertex of @c predEdge is the predecessor of a
    vertex along the shortest path to this vertex.

    The implementation of the algorithms is based on the book
    "Algorithmic graph theory" by James A. McHugh.
*/
class HXGRAPHALGORITHMS_API McDijkstraShortestPath {
public:
    /** This function computes the shortest path between the vertices
        with indices @c s and @c t. */
    static bool computeOneToOne(McDArray<McDijkstraVertex> & vertices, 
                                McDArray<McDijkstraEdge> & edges,
                                int s, int t);
    
    /** This function computes the shortest paths between the vertex
        with index @c s and all other vertices, if they are reachable
        from vertex @c s. */
    static bool computeOneToAll(McDArray<McDijkstraVertex> & vertices, 
                                McDArray<McDijkstraEdge> & edges,
                                int s);
};

#endif


