//
//  UsefulFunctions_gr1.cpp
//
//
//  Created by William Chermanne on 6/03/17.
//
//

#include "ctrl_main_gr1.h"
#include "namespace_ctrl.h"
#include "UsefulFunctions_gr1.hpp"
#include "CtrlStruct_gr1.h"
#include <math.h>
#include <stdlib.h>
#include "user_realtime.h"

NAMESPACE_INIT(ctrlGr1);

/* This function calculates the Inverse of the 3x3 matrix A and stores it in Result
 *
 * param[in] : 3x3 A, 3x3 Result
 */

void inv_mat_33(double result[3][3], double A[3][3]){

    double det = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);
    double invDet = 1.0/det;

    //Transposed matrix
    double A_t[3][3];
    int i,j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            A_t[i][j] = A[j][i];
        }
    }
    //Cofactors:
    double M_00 = A_t[1][1]*A_t[2][2] - A_t[2][1]*A_t[1][2];
    double M_01 = A_t[1][0]*A_t[2][2] - A_t[2][0]*A_t[1][2];
    double M_02 = A_t[1][0]*A_t[2][1] - A_t[1][1]*A_t[2][0];
    double M_10 = A_t[0][1]*A_t[2][2] - A_t[2][1]*A_t[0][2];
    double M_11 = A_t[0][0]*A_t[2][2] - A_t[2][0]*A_t[0][2];
    double M_12 = A_t[0][0]*A_t[2][1] - A_t[2][0]*A_t[0][1];
    double M_20 = A_t[0][1]*A_t[1][2] - A_t[1][1]*A_t[0][2];
    double M_21 = A_t[0][0]*A_t[1][2] - A_t[1][0]*A_t[0][2];
    double M_22 = A_t[0][0]*A_t[1][1] - A_t[1][0]*A_t[0][1];

    //Cofactors matrix:
    result[0][0] =   M_00 * invDet;   result[0][1] = - M_01 * invDet;   result[0][2] =   M_02 * invDet;
    result[1][0] = - M_10 * invDet;   result[1][1] =   M_11 * invDet;   result[1][2] = - M_12 * invDet;
    result[2][0] =   M_20 * invDet;   result[2][1] = - M_21 * invDet;   result[2][2] =   M_22 * invDet;
}

/* This function calculates the result of multiplication of 3x3 matrices
 * param[in] : 3x3 A, 3x3 B, 3x3 Result
 */

void mult_mat_33_33(double result[3][3], double A[3][3], double B[3][3]){
    int i,j;
    for(i = 0; i < 3; i++){
        for( j = 0; j < 3; j++){
            result[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
        }
    }
}

/* This function calculates the result of the addition of 3x3 matrices
 * param[in] : 3x3 A, 3x3 B, 3x3 Result
 */
void add_mat_33_33(double result[3][3], double A[3][3], double B[3][3]){
    int i,j;
    for(i = 0; i < 3; i++){
        for( j = 0; j < 3; j++){
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

/* This function calculates the absolute value of a number
 * param[in] : x
 * param[out] : abs(x)
 */
double absolute(double x){
    if(x < 0){
        return - x;
    }else{
        return x;
    }
}

/* This function limits a value within a specified range
 * param[in] : x,min,max
 * param[out] : x in [min,max]
 */
double limit_min_max(double x, double min, double max)
{
    double result;
    if(x < min){
        result = min;
    }else if (x > max){
        result = max;
    }else{
        result = x;
    }
    return result;
}

/* This function computes the EuclidianDistance between two points
 * param[in] : x,y of the two points
 * param[out] : double distance
 */
double EuclidianDistance (double x1,double y1,double x2,double y2)
{
    double dist= double(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
    return dist;
}

/* This function computes the ManhattanDistance between two points
 * param[in] : x,y of the two points
 * param[out] : double distance
 */
double ManhattanDistance (double x1,double y1,double x2,double y2)
{
    double dist= fabs(x1-x2)+fabs(y1-y2);
    return dist;
}

/* This function removes an element from a tabular
 * param[in] : tabular,size of the tabular, element to remove
 */
void remove_element(int* array, int sizeOfArray, int NodeToRemove)
{
  int indexToRemove;
  for (int i = 0; i < sizeOfArray; i++)
   {
     if (NodeToRemove==array[i])
     {
       indexToRemove=i;
     }
   }
   array[indexToRemove]=array[sizeOfArray-1];
}

/* This function prints all the elements of a tabular
 * param[in] : tabular,size of the tabular
 */
void printArray(int* arrayToPrint,int lengthArray)
{
  printf("Length :%d \n",lengthArray);
  printf("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
  for(int k=0;k<lengthArray;k++)
  {
    printf(" %d",arrayToPrint[k]);
  }
  printf("\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");

}

/* This function reverts a tabular
 * param[in] : tabular to revert,size of the tabular,temporary tabular that is the reverted array actually
 */
void revertArray(int* arrayToRevert,int lengthArray,int *temp)
{
  for(int i=0;i<lengthArray;i++)
  {
    temp[i]=arrayToRevert[lengthArray-1-i];
  }
}

/* This function creates the text file for the Astar structure and writes in it
 * param[in] : Astar, name of the file
 */
void createTxtFile(char *myName,Astar *A)
{
  FILE *fptr;
  fptr= fopen(myName,"w");
  int nbr_nodes=A->nbr_nodes;

  if(fptr==NULL)
  {
    printf("File does not exist!\n");
    return;
  }

  for (int i = 0; i < A->nbr_nodes; i++)
  {
    int index=A->map[i]->index;
    double x=A->map[i]->x;
    double y=A->map[i]->y;
    bool walkable = A->map[i]->walkable;
    int parent_index=A->map[i]->parent_index;

    bool isInClosedList = A->map[i]->isInClosedList;
    bool isInOpenList = A->map[i]->isInOpenList;

    double Hcost=A->map[i]->Hcost;
    double Gcost=A->map[i]->Gcost;
    double Fcost=A->map[i]->Fcost;

    bool isOnPath = A->map[i]->isOnPath;



    fprintf(fptr,"%d\t %f\t %f\t %d\t %d\t %d\t %d\t %f\t %f\t %d\t\n", index,x,y,walkable,parent_index,isInClosedList,isInOpenList,Hcost,Gcost,isOnPath);
  }

}

NAMESPACE_CLOSE();
