//
//  Odometry_gr1.hpp
//
//
//  Created by William Chermanne on 12/03/17.
//
//
#include <stdio.h>

#ifndef Odometry_gr1_hpp
#define Odometry_gr1_hpp

#include <stdio.h>
//#include "namespace_ctrl.h"
//NAMESPACE_INIT(ctrlGr1);
void StructOdometry_init(CtrlStruct *cvs);
double *xsiRWheels(CtrlStruct *cvs);
void computePosition(CtrlStruct *cvs,double *xsiRpoint);
void displayOdometry(CtrlStruct *cvs);



//NAMESPACE_CLOSE();

#endif /* Odometry_gr1_hpp */
