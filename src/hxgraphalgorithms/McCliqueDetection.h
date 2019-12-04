#ifndef MC_CLIQUE_DETECTION_H
#define MC_CLIQUE_DETECTION_H

#include <mclib/McDArray.h>
#include <mclib/McBitfield.h>

#include "api.h"

class HXGRAPHALGORITHMS_API McCliqueDetection {

public:
    static void getCliquesBasic(McDArray<int> & clique, 
                           McDArray<int> & ,
                           McDArray<int> & cand, 
                           McDArray<McBitfield> & connected,
                           int & nCliques);
    
    static void getCliquesBasicImproved(McDArray<int> & clique, 
                           McDArray<int> & notAndCand, 
                           int endNot, int endCand,
                           McDArray<McBitfield> & connected,
                           int & nCliques);
    
    static void getCliques(McDArray<int> & clique, 
                           McDArray<int> & notAndCand, 
                           int endNot, int endCand,
                           McDArray<McBitfield> & connected,
                           int & nCliques,
                           McDArray<McDArray<int> > & cliques, 
                           int minCliqueSize=1);
};

#endif
