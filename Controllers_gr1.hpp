//
//  Controllers_gr1.hpp
//
//
//  Created by William Chermanne on 27/02/17.
//
//

#include <stdio.h>
#include "CtrlStruct_gr1.h"
#ifndef Controllers_gr1_hpp
#define Controllers_gr1_hpp

//#include "namespace_ctrl.h"

//NAMESPACE_INIT(ctrlGr1);

void StructControl_init(CtrlStruct *cvs);// Initialiser la structure controle
void displayControllers(CtrlStruct *cvs);

double *PI_controller(CtrlStruct *cvs, double *ref_speed, double Kp, double Ki);

double *LowLevelController(CtrlStruct *cvs, double *ref_speed, double Kp, double Ki);

double *MiddleLevelController(CtrlStruct *cvs);

double *Wheels_reference_speed(double vref,double wref);

double Limiter(double input);


double *Kp_Ki_Computation(double overshoot, double time_response);
//NAMESPACE_CLOSE();

#endif /* Controllers_gr1_hpp */
