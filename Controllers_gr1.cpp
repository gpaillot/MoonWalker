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

void displayControllers(CtrlStruct *cvs)
{
    printf("Controllers!\n");
}
double *PI_controller(CtrlStruct *cvs, double *ref_speed, double Kp, double Ki)
{
    // memory allocation for the return value
    double *command;
    command= (double *)malloc(sizeof(double)* 2);
    
    // calling for control structure => Ki, Kp, sum_error
    
    double sum_error_right = cvs->struct_control->sum_error[0];
    double sum_error_left = cvs->struct_control->sum_error[1];
    
    // current speed
    double curr_speed_right = cvs->inputs->r_wheel_speed;
    double curr_speed_left = cvs->inputs->l_wheel_speed;
    
    //
    double delta_t = 0.01;
    double command_right, command_left;
    double curr_error_right = ref_speed[0] - curr_speed_right; // speed error right wheel definition
    double curr_error_left = ref_speed[1] - curr_speed_left; // speed error left wheeldefinition
    
    sum_error_right += curr_error_right;
    sum_error_left += curr_error_left;
    //printf("error = %f \n",sum_error_right);
    //printf("current error = %f \n",curr_error_right) ;   
    cvs->struct_control->sum_error[0] = sum_error_right;
    cvs->struct_control->sum_error[1] = sum_error_left;
    
    command_right = Kp*curr_error_right + Ki*sum_error_right*delta_t;
    command_left = Kp*curr_error_left + Ki*sum_error_left*delta_t;
    
    command[0] = command_right;
    command[1] = command_left;
    
    return command;
    
}

double *LowLevelController(CtrlStruct *cvs, double *ref_speed, double Kp, double Ki)
{
    double u_left_prime, u_right_prime, kphi, K;
    kphi=26.1e-3;
    K =1;
    double *pi_contr = (double*)malloc(sizeof(double)*2);
    double *u= (double*)malloc(sizeof(double)*2);
    pi_contr = PI_controller(cvs,ref_speed,Kp,Ki);
    
    // limiter stage
    u_right_prime = Limiter(pi_contr[0]);
    u_left_prime = Limiter(pi_contr[1]);
    
    // Back EMF compenstation stages
    u_right_prime  = u_right_prime + (kphi*14/K)*(cvs->inputs->r_wheel_speed);
    u_left_prime = u_left_prime + (kphi*14/K)*(cvs->inputs->l_wheel_speed);
    
    // second limiter stage
    u_right_prime = Limiter(u_right_prime);
    u_left_prime = Limiter(u_left_prime);
    
    // variable assignation
    u[0] = u_right_prime;
    u[1] = u_left_prime;
    
    return u;
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
    if(input >90)
    {
        output = 90;
    }
    else if(input < -90)
    {
        output = -90;
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
    double J=0.58e-6;
    double Kv=9.66e-6;
    double K = 1;
    
    double t_mechanical = J/Kv; // mechanical time constant
    double xsi = sqrt((CARRE(log(overshoot)))/((CARRE(PI))+ CARRE(log(overshoot))));
    double wn = 4/(xsi*time_response);
    double Ki = ((CARRE(wn))*Kv*Ra*t_mechanical)/(K*kphi);
    double Kp =((2*xsi*wn)*(Ra*Kv*t_mechanical) - Ra*Kv)/(K*kphi);
    
    output[0] = Kp;
    output[1] = Ki;
    return output;
    
    
}
