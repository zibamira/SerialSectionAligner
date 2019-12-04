/////////////////////////////////////////////////////////////////
//
// McCliqueDetection.h
//
// Main Authors: Baum
// 
/////////////////////////////////////////////////////////////////

#ifndef MC_CLIQUE_DETECTION_H
#define MC_CLIQUE_DETECTION_H

#include <mclib/McDArray.h>
#include <mclib/McBitfield.h>

#include "api.h"

class POINTMATCHING_API McCliqueDetection {

public:
    static void computeCliquesBasic(
                    McDArray<int>        & clique, 
                    McDArray<int>        & notList,
                    McDArray<int>        & cand, 
                    McDArray<McBitfield> & connected,
                    int                  & nCliques);
    
    static void computeCliquesBasicImproved(
                    McDArray<int>        & clique, 
                    McDArray<int>        & notAndCand, 
                    int                    endNot, 
                    int                    endCand,
                    McDArray<McBitfield> & connected,
                    int                  & nCliques);
    
    static bool computeCliquesBronKerbosch(
                    McDArray<int>            & clique, 
                    McDArray<int>            & notAndCand, 
                    int                        endNot, 
                    int                        endCand,
                    McDArray<McBitfield>     & connected,
                    int                      & nCliques,
                    McDArray<McDArray<int> > & cliques, 
                    int                        minCliqueSize=1,
                    int                        maxNumCliques=-1);
};

#endif
