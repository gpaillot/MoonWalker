//
//  UsefulFunctions_gr1.hpp
//
//
//  Created by William Chermanne on 6/03/17.
//
//

#ifndef UsefulFunctions_gr1_hpp
#define UsefulFunctions_gr1_hpp

#include <stdio.h>


NAMESPACE_INIT(ctrlGr1);

void inv_mat_33(double result[3][3], double A[3][3]);
void mult_mat_33_33(double result[3][3], double A[3][3], double B[3][3]);
void add_mat_33_33(double result[3][3], double A[3][3], double B[3][3]);
double absolute(double x);
double limit_min_max(double x, double min, double max);
double EuclidianDistance (double x1,double y1,double x2,double y2);
void remove_element(int* array, int sizeOfArray, int NodeToRemove);
void printArray(int* arrayToPrint,int lengthArray);
void revertArray(int* arrayToRevert,int lengthArray,int *temp);
void createTxtFile(char *myName,Astar *A);
double ManhattanDistance (double x1,double y1,double x2,double y2);


NAMESPACE_CLOSE();

#endif /* UsefulFunctions_gr1_hpp */
