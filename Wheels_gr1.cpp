//
//  Wheels_gr1.cpp
//
//
//  Created by William Chermanne on 21/02/17.
//
//


/*!
 * \file Tower_gr1.cc
 * \brief File to use the tower
 */

#include "Wheels_gr1.hpp"
#include "CtrlStruct_gr1.h"
#include <math.h>
#include <stdlib.h>

#define PI 3.1415926535897932384626


// To do : mettre delta_t dans la structure!

/* This function displays some useful informations about the wheels
 *
 * param[in] : cvs controller main structure
 *
 */
void displayWheels(CtrlStruct *cvs) {


    CtrlIn *ivs;
    CtrlOut *ovs;
    StructWheels *wheels;

    ivs = cvs->inputs;
    ovs = cvs->outputs;
    wheels = cvs->struct_wheels;

    double *position;
    position = (double *)malloc(sizeof(double)* 2);

    double *tempxsirpoint = xsiRWheels(cvs);
    position = (double *)malloc(sizeof(double)* 3);

    position = computePosition(cvs,tempxsirpoint);
    //printf("Position x = %f \n",wheels->x_t);
    //printf("Position y = %f \n",wheels->y_t);
    //printf("Position theta = %f\n",wheels->theta_t);

}




/* This function computes the wheels velocities x_dot,y_dot and theta_dot in the robot space
 *
 * param[in] : cvs controller main structure
 * out : [x_dot,y_dot,theta_dot]
 */
double *xsiRWheels(CtrlStruct *cvs)
{
    double *xsiRpoint;
    xsiRpoint = (double *)malloc(sizeof(double)* 3);

    double radius = 0.03;
    double l = 0.1125;

    // phi_1 = roue droite, phi_2 = roue gauche
    xsiRpoint[0] = (radius*cvs->inputs->r_wheel_speed)*0.5 + (radius*cvs->inputs->l_wheel_speed)*0.5;
    xsiRpoint[1] = 0;
    xsiRpoint[2] = (radius*cvs->inputs->r_wheel_speed)/(2*l) - (radius*cvs->inputs->l_wheel_speed)/(2*l);

    return xsiRpoint;
}


/* This function computes the position of the robot (=odometry)
 *
 * param[in] : cvs controller main structure, table with speeds of the wheels
 * out : [xr,yr]
 */
double *computePosition(CtrlStruct *cvs,double *xsiRpoint)
{
    double *position;
    position = (double *)malloc(sizeof(double)* 3);

    double v = xsiRpoint[0];
    double w = xsiRpoint[2];

    double delta_t = 0.023; // to be modified 

    double theta = w*delta_t + cvs->struct_wheels->theta_t;
    if(theta>PI)
    {
      theta=theta-2*PI;
    }
    if(theta<-PI)
    {
        theta=theta+2*PI;
    }
    cvs->struct_wheels->theta_t = theta;

    double x = v*cos(theta)*delta_t + cvs->struct_wheels->x_t;
    double y = v*sin(theta)*delta_t + cvs->struct_wheels->y_t;

    position[0]=x;
    position[1]=y;

    cvs->struct_wheels->x_t = x;
    cvs->struct_wheels->y_t = y;

    return position;
}

