#ifndef KRUSKAL_TREE_CLUSTERER_H
#define KRUSKAL_TREE_CLUSTERER_H

#include <mclib/McDArray.h>
#include "api.h"
#include <math.h>

class Node;
class Edge;
class TreeList;

class HXGRAPHALGORITHMS_API KruskalTreeClusterer {
public:
	// Main function: returning for each node a id of its cluster (-1 if not clustered)
	// theNodes and theEdges dexcribing the graph to be clustered
	McDArray<McDArray<int> > clusterGraph(McDArray<McDArray<Node *> > theNodes, McDArray<Edge *> theEdges);

private:

	// weight separation, edges weighted higher then splitter, wont be used
	double splitter;

	// Kruskal Tree Clustering Algorithm
	McDArray<TreeList *> getMinimalSpanningTree(McDArray<Edge *> edges);

	// fuehrt den Algorithmus von Kruskal aus
	McDArray<TreeList *> spanTree(McDArray<Edge *> edges);

	// sucht in allen bisherigen Baeumen ob dieser Knoten darin vorkam und gibt
	// den Index des Baumes zurueck bei dem es vorkam
	int getIndexOfNode(McDArray<TreeList *> tmpTrees, Node * n);

	// sucht in einem Baum ob der Knoten darin vorkommt
	int findEdge(TreeList * tList, Node * n);
};


class Node {
public:
	int x;	
	int y;
	
	int index;
	
	McDArray<double> weights;
	
	Node(int xx, int yy, int idx, McDArray<double> vals) {
		x = xx;
		y = yy;
		
		index = idx;
		
		weights = vals;
	}
};

class Edge {
public:
	Node * from;
	Node * to;
	
	double weight;
	
	Edge(Node * fr, Node * t) {
		from = fr;
		to = t;

		int i=0;
		weight = 0;
		for(i=0; i<fr->weights.size(); i++) {
			weight += ((from->weights[i]-to->weights[i])*(from->weights[i]-to->weights[i]));
		}	

		weight = sqrt(weight);

		double half = sqrt((double)fr->weights.size()); 
		if(weight>(half/2)) {
			weight = half - weight;
		}
	}
};

class TreeList {	
public:
	McDArray<int> indices;		
	McDArray<Edge *> allEdges;	
	
	int depth;	
	
	// initialisiert den Gesamten Baum
	TreeList(Edge * e) {		
		allEdges.append(e);
				
		indices.append(e->from->index);
		indices.append(e->to->index);		
		
		depth = 1;
	}
	
	void joinTrees(TreeList * tList) {
		for(int i=0; i<tList->depth-1; i++) {
			appendEdge(tList->allEdges[i]);
		}
	}
	
	void appendEdge(Edge  *e) {
		allEdges.append(e);
		indices.append(e->from->index);
		indices.append(e->to->index);
		
		depth++;
	}
	
	
};

#endif



