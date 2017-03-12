//
//  Wheels_gr1.hpp
//
//
//  Created by William Chermanne on 20/02/17.
//
//
/*!
 * \file filename_grX.h
 * \brief File description
 */
#include "CtrlStruct_gr1.h"
#include <stdio.h>
#ifndef Wheels_gr1_hpp
#define Wheels_gr1_hpp

// #include "my_own_headers.h" // adapt it with your headers

 // where X should be replaced by your group number

// add your extra code (prototypes, macros...) here

void displayWheels(CtrlStruct *cvs);
double *Wheels_reference_speed(double vref,double wref);
double *xsiRWheels(CtrlStruct *cvs);
double *computePosition(CtrlStruct *cvs, double *xsiRpoint);



#endif // end of header guard
