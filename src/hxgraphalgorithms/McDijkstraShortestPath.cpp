#include <mclib/McBitfield.h>

#include "McDijkstraShortestPath.h"

bool McDijkstraShortestPath::computeOneToOne(
                                     McDArray<McDijkstraVertex> & vertices, 
                                     McDArray<McDijkstraEdge> & edges,
                                     int s, int t)
{
    // If a vertex has not been reached yet, it needs to be inserted
    // into the heap. If the was previously reached it is either still
    // in the heap and might have to be modified, or the shortest path
    // to this vertex has already been found. In the latter case it
    // has already been removed from the heap.
    McBitfield reached(vertices.size());
    reached.unsetAll();
    
    int i;
    for ( i=0; i<vertices.size(); i++ ) {
        vertices[i].idx = i;
        // initial distance is set to some large value
        vertices[i].distance = 1000000.0;
        // initially there is no predecessor edge 
        vertices[i].predEdge = 0;
    }
    
    // distance of start vertex from itself is trivially 0.0
    vertices[s].distance = 0.0;
    reached.set(s);
    
    McFHeap<McDijkstraVertex> fheap;
    fheap.insert(&vertices[s]);
    
    int u, v; 
    double newDistance;
    McDijkstraVertex * minElem;
    McDijkstraEdge * currEdge;

    // do ... while Fibonacci heap is not empty or vertex t is reached
    // (break while loop in the latter case)
    while ( minElem = fheap.getMin() ) {
        if ( minElem->idx == t ) {
            // end vertex has been reached, i.e., shortest path has
            // been found
            break;
        }
        
        // u is current vertex
        u = minElem->idx;
        fheap.deleteMin();

        // iterate over all edges leaving vertex u
        currEdge = minElem->edges.first();
        while ( currEdge ) {
            v = currEdge->endVertex;
            newDistance = vertices[u].distance + currEdge->edgeLength;
            
            if ( !reached[v] ) {
                // v is reached for the first time
                vertices[v].distance = newDistance;
                vertices[v].predEdge = currEdge;
                
                reached.set(v);
                fheap.insert(&vertices[v]);
            } else {
                // v has been reached before
                
                if ( vertices[v].distance > newDistance ) {
                    // only do something if newDistance is shorter
                    // than old one
                    fheap.deleteElem(&vertices[v]);
                    
                    vertices[v].distance = newDistance;
                    vertices[v].predEdge = currEdge;
                    
                    fheap.insert(&vertices[v]);                    
                }
            }
            
            currEdge = minElem->edges.succ(currEdge);
        }
    }
    
    if ( vertices[t].predEdge == 0 ) {
        // no shortest path from s to t could be found, i.e., there is none
#ifndef NDEBUG
        printf("Error: There is no path from vertex %d to vertex %d.\n", s, t);
#endif
        return false;
    }
    
#ifndef NDEBUG
    printf("path (back->beginning): %d ", t);
    McDijkstraEdge * predEdge = vertices[t].predEdge;
    while ( predEdge->startVertex != s ) {
        printf("%d ", predEdge->startVertex);
        predEdge = vertices[predEdge->startVertex].predEdge;
    }
    printf("%d\n", predEdge->startVertex); fflush(stdout);
#endif
    
    return true;
}

bool McDijkstraShortestPath::computeOneToAll(
                                     McDArray<McDijkstraVertex> & vertices, 
                                     McDArray<McDijkstraEdge> & edges,
                                     int s)
{
    /* Documentation is the same as above. The only difference between
       the two functions is that the while loop is only left when
       Fibonacci heap is empty. */ 

    McBitfield reached(vertices.size());
    reached.unsetAll();
    
    McFHeap<McDijkstraVertex> fheap;
    int i;
    for ( i=0; i<vertices.size(); i++ ) {
        vertices[i].idx = i;
        vertices[i].distance = 1000000.0;
        vertices[i].predEdge = 0;
    }
    
    vertices[s].distance = 0.0;
    reached.set(s);

    fheap.insert(&vertices[s]);
    
    int u, v; 
    double newDistance;
    McDijkstraVertex * minElem;
    McDijkstraEdge * currEdge;
    while ( minElem = fheap.getMin() ) {
        u = minElem->idx;
        fheap.deleteMin();

        currEdge = minElem->edges.first();
        while ( currEdge ) {
            v = currEdge->endVertex;
            newDistance = vertices[u].distance + currEdge->edgeLength;

            if ( !reached[v] ) {
                vertices[v].distance = newDistance;
                vertices[v].predEdge = currEdge;
                
                reached.set(v);
                fheap.insert(&vertices[v]);
            } else {
                
                if ( vertices[v].distance > newDistance ) {
                    fheap.deleteElem(&vertices[v]);
                    
                    vertices[v].distance = newDistance;
                    vertices[v].predEdge = currEdge;
                    
                    fheap.insert(&vertices[v]);                    
                }
            }

            currEdge = minElem->edges.succ(currEdge);
        }
    }
    
#ifndef NDEBUG
    for ( i=0; i<vertices.size(); i++ ) {
        printf("vertex %d: %f\n", i, vertices[i].distance);
    }
    fflush(stdout);
#endif
    
    return true;
}
