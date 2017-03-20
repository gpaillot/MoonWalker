//
//  Tower_gr1.hpp
//
//
//  Created by William Chermanne on 20/02/17.
//
//
/*!
 * \file filename_grX.h
 * \brief File description
 */

#include <stdio.h>
#ifndef Tower_gr1_hpp
#define Tower_gr1_hpp

#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number

// add your extra code (prototypes, macros...) here
void StructTower_init(CtrlStruct *cvs);
double angle_opponent(CtrlStruct *cvs);
double distance_opponent(CtrlStruct *cvs);
double *anglesTable_fixed(CtrlStruct *cvs);
void tower_display(CtrlStruct *cvs);
double cotan(double a);
double *triangulation(double alpha1, double alpha2, double alpha3,double x1, double y1, double x2, double y2, double x3, double y3);
double adjust_value_to_bounds( double cot ,double cot_max );


NAMESPACE_CLOSE();

#endif // end of header guard
