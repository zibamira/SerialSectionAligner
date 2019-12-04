#ifndef MC_MIN_SPANNING_TREE_H
#define MC_MIN_SPANNING_TREE_H

#include <mclib/McDArray.h>
#include <mclib/McVec2i.h>
#include <mclib/internal/McFHeap.h>
#include <mclib/internal/McSorter.h>

template <class T>
class IndexByScoreComparatorTemplate {
    T * theValues;
    
public:
    IndexByScoreComparatorTemplate(T * values) : theValues(values) {}
    
    int operator()(int first, int second)
    { 
        if(theValues[first]<theValues[second])
            return -1;
        if(theValues[first]>theValues[second])
            return 1;
        
        return 0;
    }
};

template <class T>
class McMSTHeapElement : public McFHeapElement {
public:
    // Graph variables.
    int edgeId;
    T   edgeWeight;
    
    /// Comparison.
    int operator<(const McMSTHeapElement<T> & elem)
    { return (edgeWeight < elem.edgeWeight); }
};

/** 
    The implementation of the algorithms is based on the book
    "Algorithmic graph theory" by James A. McHugh.
*/ 
template <class T>
class McMinSpanningTree {
public:
    /** This function takes as input a list of edges, determining the
        graph, a list of weights corresponding to the edges, and the
        number of vertices of the graph. The output is the minimum
        spanning tree of the given graph stored as a list of edge
        indices. In order to find the next edge with smallest weight a
        Fibonacci heap is used. */
    bool computeMinSpanningTreeFHeap(const McDArray<McVec2i> & edges,
                                     const McDArray<T> & weight,
                                     int nVertices,
                                     McDArray<int> & edgeIdOfMST);
    /** Same as above, only that for finding the next edge with
        smallest weight a sorted list is used. Theoretically, using a
        Fibonacci heap is faster. However, in practice, depending on
        the size of the graph, it might be faster to use this
        function. */
    bool computeMinSpanningTreeQSort(const McDArray<McVec2i> & edges, 
                                     const McDArray<T> & weight, 
                                     int nVertices, 
                                     McDArray<int> & edgeIdOfMST);
    
private:
    class McMSTVertex {
    public:
        // Unary tree variables.
        McMSTVertex * treeParent;
        int treeSize;
        
        // Default constructor.
        McMSTVertex() { treeParent = 0; treeSize = 1; }

        McMSTVertex * root() {
            McMSTVertex * currVertex = this;
            while ( currVertex->treeParent != 0 ) {
                currVertex = currVertex->treeParent;
            }
            
            return currVertex;
        }
        
        // Merge two unary trees.
        void merge(McMSTVertex * root2) {
            McMSTVertex * root1 = this;
            if ( root1->treeSize > root2->treeSize ) {
                root2->treeParent = root1;
                root1->treeSize += root2->treeSize;
            } else {
                root1->treeParent = root2;
                root2->treeSize += root1->treeSize;
            }
        }
    };
};

template < class T >
bool McMinSpanningTree<T>::computeMinSpanningTreeFHeap(
                                         const McDArray<McVec2i> & edges,
                                         const McDArray<T> & weight,
                                         int nVertices,
                                         McDArray<int> & edgeIdsOfMST)
{
    McFHeap<McMSTHeapElement<T> > fheap;
    McDArray<McMSTHeapElement<T> > heapElements(edges.size());
    int i;
    for ( i=0; i<heapElements.size(); i++ ) {
        heapElements[i].edgeId     = i;
        heapElements[i].edgeWeight = weight[i];
        fheap.insert(&heapElements[i]);
    }
    
    McDArray<McMSTVertex> vertices(nVertices);
    
    McMSTHeapElement<T> * minElem;
    McMSTVertex * treeRoot1;
    McMSTVertex * treeRoot2;
    for ( i=0; i<heapElements.size(); i++ ) {
        minElem = fheap.getMin();
        fheap.deleteMin();
        
        treeRoot1 = vertices[edges[minElem->edgeId][0]].root();
        treeRoot2 = vertices[edges[minElem->edgeId][1]].root();
        
        if ( treeRoot1 != treeRoot2 ) {
            treeRoot1->merge(treeRoot2);
            edgeIdsOfMST.append(minElem->edgeId);
            if ( edgeIdsOfMST.size() == nVertices-1 ) {
                break;
            }
        }
    }

    if ( edgeIdsOfMST.size() < nVertices-1 ) 
        return false;
    return true;
}

template < class T >
bool McMinSpanningTree<T>::computeMinSpanningTreeQSort(
                                         const McDArray<McVec2i> & edges,
                                         const McDArray<T> & weight,
                                         int nVertices,
                                         McDArray<int> & edgeIdsOfMST)
{
    McDArray<int> indices(edges.size());
    int i;
    for ( i=0; i<indices.size(); i++ ) {
        indices[i] = i;
    }
    IndexByScoreComparatorTemplate<T> comparator((T*)&(weight[0])); 
    sort(&(indices[0]), indices.size(), comparator);

    McDArray<McMSTVertex> vertices(nVertices);
    
    McMSTVertex * treeRoot1;
    McMSTVertex * treeRoot2;
    for ( i=0; i<indices.size(); i++ ) {
        
        treeRoot1 = vertices[edges[indices[i]][0]].root();
        treeRoot2 = vertices[edges[indices[i]][1]].root();
        
        if ( treeRoot1 != treeRoot2 ) {
            treeRoot1->merge(treeRoot2);
            edgeIdsOfMST.append(indices[i]);
            if ( edgeIdsOfMST.size() == nVertices-1 ) {
                break;
            }
        }
    }

    if ( edgeIdsOfMST.size() < nVertices-1 ) 
        return false;
    return true;
}    

#endif


