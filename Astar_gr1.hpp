//
//  Astar_gr1.hpp
//
//
//  Created by William Chermanne on 4/03/17.
//
//
#include <stdio.h>
#ifndef Astar_gr1_hpp
#define Astar_gr1_hpp


//NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number

void Astar_init(CtrlStruct *cvs); // Pour initialiser le Astar
void Astar_free(Astar *A);
void Create_map(Astar *A, int lign_x,int lign_y);
void Create_map_hardcoded(Astar *A);

void Create_path(Astar *A, int indexNodeStart, int indexNodeTarget);
void RetrieveReversedPath(Astar *A,int TargetNode, int* path);
void ComputeLengthPath(Astar *A,int TargetNode);
void RetrievePath(Astar *A,int TargetNode,int *path);
void SelectNodesPath(Astar *A,int *path);
void CreatePathOnlyTurns(Astar *A);


// Nodes operation
void add_open_list(Astar *A,Node *N);
void current_to_closed_list(Astar *A,Node *N);
int lowest_cost_open_list(Astar *A);
int FindNearestNode(Astar *A,double xTarget,double yTarget);
double slopeBetweenNodes(Astar *A,int indexNode1, int indexNode2);

#endif /* Astar_gr1_hpp */
