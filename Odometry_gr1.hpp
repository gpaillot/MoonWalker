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

void StructOdometry_init(CtrlStruct *cvs);
void xsiRWheels(CtrlStruct *cvs, int unit);
void computePosition(CtrlStruct *cvs);
void displayOdometry(CtrlStruct *cvs);



//NAMESPACE_CLOSE();

#endif /* Odometry_gr1_hpp */
