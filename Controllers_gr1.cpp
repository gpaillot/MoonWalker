//
//  Controllers_gr1.cpp
//
//
//  Created by William Chermanne on 27/02/17.
//
//



//#include "ctrl_main_gr1.h"
//#include "namespace_ctrl.h"
#include "Controllers_gr1.hpp"
#include "CtrlStruct_gr1.h"
#include <math.h>
#include <stdlib.h>
//#include "user_realtime.h"

#define PI 3.141592653
#define CARRE(X) (X)*(X)

///NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number
void StructControl_init(CtrlStruct *cvs) // Initialiser la structure controle
{
  cvs->struct_control->sum_error = (double *) malloc(2*sizeof(double));
  cvs->struct_control->currentError = (double *) malloc(2*sizeof(double));
  cvs->struct_control->previousError = (double *) malloc(2*sizeof(double));
  cvs->struct_control->previousCommand = (double *) malloc(2*sizeof(double));
  cvs->struct_control->previousCommandLtd = (double *) malloc(2*sizeof(double));
  cvs->struct_control->Kp = 0.0; //to be computed !!!!!
  cvs->struct_control->Ki = 0.0; //to be computed !!!!!
  cvs->struct_control->sum_error[0] = 0.0; // right wheel
  cvs->struct_control->sum_error[1] = 0.0; // left wheel
  cvs->struct_control->Kt = 30.0;         // should not change
  cvs->struct_control->Tsample = 0.001;   // sampling period (1ms)
  cvs->struct_control->currentError[0] = 0.0;      // initial errors
  cvs->struct_control->currentError[1] = 0.0;
  cvs->struct_control->previousError[0] = 0.0;
  cvs->struct_control->previousError[1] = 0.0;
  cvs->struct_control->previousCommand[0] = 0.0;   // initial commands
  cvs->struct_control->previousCommand[1] = 0.0;
  cvs->struct_control->previousCommandLtd[0] = 0.0;
  cvs->struct_control->previousCommandLtd[1] = 0.0;

}

void displayControllers(CtrlStruct *cvs)
{
    printf("Controllers!\n");
}
double *PI_controller(CtrlStruct *cvs, double *ref_speed, double Kp, double Ki)
{
    // memory allocation for the return value
       double *command;
    command = (double *)malloc(sizeof(double)* 2);

    // current speed
    double curr_speed_right = cvs->inputs->r_wheel_speed;
    double curr_speed_left = cvs->inputs->l_wheel_speed;
    // errors
    cvs->struct_control->currentError[0] = ref_speed[0] - curr_speed_right;
    cvs->struct_control->currentError[1] = ref_speed[1] - curr_speed_left;

    // initializations and short names
    double command_right, command_left;
    double Tspl = cvs->struct_control->Tsample;
    double CerrR = cvs->struct_control->currentError[0];
    double CerrL = cvs->struct_control->currentError[1];
    double PerrR = cvs->struct_control->previousError[0];
    double PerrL = cvs->struct_control->previousError[1];
    double commR = cvs->struct_control->previousCommand[0];
    double commL = cvs->struct_control->previousCommand[1];
    double commLtdR = cvs->struct_control->previousCommandLtd[0];
    double commLtdL = cvs->struct_control->previousCommandLtd[1];
    double Kt = cvs->struct_control->Kt;

    command_right = ((Kp + Ki*Tspl)*CerrR - Kp*PerrR + Tspl*Kt*commLtdR + commR)/(Kt*Tspl + 1);
    command_left  = ((Kp + Ki*Tspl)*CerrL - Kp*PerrL + Tspl*Kt*commLtdL + commL)/(Kt*Tspl + 1);

    command[0] = command_right;
    command[1] = command_left;

    // assigning "previous" values to "current" ones
    cvs->struct_control->previousError[0] = CerrR;
    cvs->struct_control->previousError[1] = CerrL;
    cvs->struct_control->previousCommand[0] = command_right;
    cvs->struct_control->previousCommand[1] = command_left;
    cvs->struct_control->previousCommandLtd[0] = Limiter(command_right);
    cvs->struct_control->previousCommandLtd[1] = Limiter(command_left);
    return command;
    
}

double *LowLevelController(CtrlStruct *cvs, double *ref_speed, double Kp, double Ki)
{
    double u_left_prime, u_right_prime, kphi, K;
    kphi=26.1e-3;
    K =0.98;
    double *pi_contr = (double*)malloc(sizeof(double)*2);
    double *u= (double*)malloc(sizeof(double)*2);
    pi_contr = PI_controller(cvs,ref_speed,Kp,Ki);

    // limiter stage
    u_right_prime = Limiter(pi_contr[0]);
    u_left_prime = Limiter(pi_contr[1]);
    free(pi_contr);
    // Back EMF compenstation stages
    u_right_prime  = u_right_prime + (kphi*14/K)*(cvs->inputs->r_wheel_speed);
    u_left_prime = u_left_prime + (kphi*14/K)*(cvs->inputs->l_wheel_speed);

    // second limiter stage
    //u_right_prime = Limiter(u_right_prime);
    //u_left_prime = Limiter(u_left_prime);

    // variable assignation
    u[0] = u_right_prime;
    u[1] = u_left_prime;
}

/* This function computes the forward and rotational speed to apply to reach the goal coordinates
 *
 * param[in] : cvs controller main structure, table with speeds of the wheels
 * out : [xr,yr]
 */

double *MiddleLevelController(CtrlStruct *cvs)
{
  double *speeds;
  speeds= (double *)malloc(sizeof(double)* 2);

  speeds[0]=0; // Forward speed
  speeds[1]=0; // Rotational speed
  return speeds;
}



/* This function computes the command to each wheel with a reference vref and wref
 *
 * param[in] : cvs controller main structure, table with speeds of the wheels
 * out : [xr,yr]
 */
double *Wheels_reference_speed(double vref,double wref)
{
    double l = 0.1125; // In meters!
    double r = 0.03;

    double *speeds;
    speeds= (double *)malloc(sizeof(double)* 2);


    double right_speed = (vref+l*wref)/r;
    double left_speed = (vref-l*wref)/r;

    speeds[0] = right_speed;
    speeds[1] = left_speed;

    return speeds;
}
/*
 * Limiter block 
 * => input : infinite range
 * => output bounded range between -100 and 100
 */

double Limiter(double input)
{
    double output;
    if(input >50)
    {
        output = 50;
    }
    else if(input < -50)
    {
        output = -50;
    }
    else
    {
        output = input;
    }
    return output;
    
}

/* computation of Kp and Ki for the low_level controller
 * designed for a Faulhaber 2342 024CR motor
 * assumption : T_electrical = 0
 *  output_0 = Kp; output_1 = Ki
 */

    
double *Kp_Ki_Computation(double overshoot, double time_response)
{
    double *output = (double*) malloc(sizeof(double)*2);
    double Ra =7.1; // résitance de l'induit
    double La=(0.265e-3); // inducantce de l'induit
    double kphi=26.1e-3;
    double J=0.58e-6+(1.607e-4/10);
    double Kv=9.66e-6;
    double K = 0.98;

    double t_mechanical = J/Kv; // mechanical time constant
    double xsi = sqrt((CARRE(log(overshoot)))/((CARRE(PI))+ CARRE(log(overshoot))));
    double wn = 4/(xsi*time_response);
    double Ki = ((CARRE(wn))*Kv*Ra*t_mechanical)/(K*kphi);
    double Kp =((2*xsi*wn)*(Ra*Kv*t_mechanical) - Ra*Kv)/(K*kphi);

    output[0] = 14*Kp; // 14 is the reduction ratio
    output[1] = Ki;
    return output;

}
