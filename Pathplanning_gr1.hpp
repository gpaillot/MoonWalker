//
//  Pathplanning_gr1.hpp
//
//
//  Created by William Chermanne on 28/02/17.
//
//

#include <stdio.h>
#ifndef Pathplanning_gr1_hpp
#define Pathplanning_gr1_hpp
#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr1);
void StructPathPlanning_init(CtrlStruct *cvs);
double HighLevelController(CtrlStruct *cvs, double *goalCoordinates);

NAMESPACE_CLOSE();

#endif /* Pathplanning_gr1_hpp */
