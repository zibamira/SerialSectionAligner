#include "McCliqueDetection.h"

void McCliqueDetection::getCliquesBasic(McDArray<int> & clique, 
                                        McDArray<int> & notList,
                                        McDArray<int> & cand, 
                                        McDArray<McBitfield> & connected,
                                        int & nCliques)
{
    int i;
    McDArray<int> notList2, cand2;
    if ( notList.size()==0 && cand.size()==0 ) {
        nCliques++;
        // printf("clique = ");
        // for ( i=0; i<clique.size(); i++ ) {
        //     printf("%d ", clique[i]);
        // }
        // printf("\n"); fflush(stdout);
        return;
    } 
    
    while ( cand.size()>0 ) {
        clique.append(cand[0]);
        cand2.clear();
        for ( i=1; i<cand.size(); i++ ) {
            if ( connected[cand[0]][cand[i]] ) 
                cand2.append(cand[i]);
        }
        notList2.clear();
        for ( i=0; i<notList.size(); i++ ) {
            if ( connected[cand[0]][notList[i]] ) 
                notList2.append(notList[i]);
        }
        getCliquesBasic(clique, notList2, cand2, connected, nCliques);
        
        notList.append(cand[0]);
        cand.remove(0);
        clique.removeLast();
    }
}

void McCliqueDetection::getCliquesBasicImproved(McDArray<int> & clique, 
                                         McDArray<int> & notAndCand, 
                                         int endNot, int endCand,
                                         McDArray<McBitfield> & connected,
                                         int & nCliques)
{
    int i, endNot2, endCand2, cand;
    McDArray<int> notAndCand2;
    if ( endCand == 0 ) {
        nCliques++;
        // printf("clique = ");
        // for ( i=0; i<clique.size(); i++ ) {
        //     printf("%d ", clique[i]);
        // }
        // printf("\n"); fflush(stdout);
        return;
    } 
    
    while ( endNot != endCand ) {
        cand = notAndCand[endNot];
        clique.append(cand);
        notAndCand2.clear();
        
        endNot2 = 0;
        for ( i=0; i<endNot; i++ ) {
            if ( connected[cand][notAndCand[i]] ) {
                notAndCand2.append(notAndCand[i]);
                endNot2++;
            }
        }
        endCand2 = endNot2;
        for ( i=endNot+1; i<endCand; i++ ) {
            if ( connected[cand][notAndCand[i]] ) {
                notAndCand2.append(notAndCand[i]);
                endCand2++;
            }
        }
        getCliquesBasicImproved(clique, notAndCand2, endNot2, endCand2, 
                         connected, nCliques);

        clique.removeLast();
        endNot++;
    }
}

void McCliqueDetection::getCliques(McDArray<int> & clique, 
                                   McDArray<int> & notAndCand, 
                                   int endNot, int endCand,
                                   McDArray<McBitfield> & connected,
                                   int & nCliques,
                                   McDArray<McDArray<int> > & cliques, 
                                   int minCliqueSize)
{
    McDArray<int> notAndCand2;
    
    int i, j, endNot2, endCand2, count, pos, fixp, p, s, sel;
    int minNod = endCand;
    int nod = 0;
    
    for ( i=0; i<endCand && minNod!=0; i++ ) {
        p = notAndCand[i];
        count = 0;
        for ( j=endNot; j<endCand && count<minNod; j++ ) {
            if ( !connected[p][notAndCand[j]] ) { 
                pos = j; // position of potential candidate
                count++; // increase number of disconnections
            }
        }
        if ( count < minNod ) {
            fixp = p;
            minNod = count;
            if ( i<endNot ) {
                s = pos;
            } else {
                s = i;
                nod = 1;
            }
        }
    }

    for ( nod=minNod+nod; nod>0; nod-- ) {
        p = notAndCand[s];
        notAndCand[s] = notAndCand[endNot];
        sel = notAndCand[endNot] = p;
        notAndCand2.resize(notAndCand.size());
        endNot2 = 0;
        for ( i=0; i<endNot; i++ ) {
            if ( connected[sel][notAndCand[i]] ) {
                notAndCand2[endNot2] = notAndCand[i];
                endNot2++;
            }
        }
        endCand2 = endNot2;
        for ( i=endNot+1; i<endCand; i++ ) {
            if ( connected[sel][notAndCand[i]] ) {
                notAndCand2[endCand2] = notAndCand[i];
                endCand2++;
            }
        }
        notAndCand2.resize(endCand2);
        clique.append(sel);
        
        if ( endCand2 == 0 ) {
            if ( clique.size() >= minCliqueSize ) {
                nCliques++;
                cliques.append(clique);
            }
        } else if ( endNot2 < endCand2 ) {
            if ( clique.size()+(endCand2-endNot2) >= minCliqueSize ) {
                getCliques(clique, notAndCand2, endNot2, endCand2, 
                    connected, nCliques, cliques, minCliqueSize);
            }
        } 
        clique.removeLast();
        endNot++;
        
        if ( nod>1 ) {
            s = endNot;
            while ( connected[fixp][notAndCand[s]] ) {
                s++;
            }
        }
    }
}

