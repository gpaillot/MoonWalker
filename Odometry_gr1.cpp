//
//  Odometry_gr1.cpp
//
//
//  Created by William Chermanne on 12/03/17.
//
//

#include "MyIncludes_gr1.h"
#include <math.h>

#define PI 3.1415926535897932384626

//NAMESPACE_INIT(ctrlGr1);

/* This function initializes the StructWheels structure
 *
 * param[in] : cvs controller main structure
 */

void StructOdometry_init(CtrlStruct *cvs)
{
  //////////////////////   Structure Wheels //////////////////////

  cvs->struct_odometry->counter = 0;
  cvs->struct_odometry->x_t = -850;
  cvs->struct_odometry->y_t = -1150;
  cvs->struct_odometry->theta_t = PI/2;
  cvs->struct_odometry->prev_speed[0] = 0.0;
  cvs->struct_odometry->prev_speed[1] = 0.0;
  cvs->struct_odometry->prev_speed[0] = 0.0;
  cvs->struct_odometry->prev_speed[1] = 0.0;
  cvs->struct_odometry->xsiRpoint[0] = 0.0;
  cvs->struct_odometry->xsiRpoint[1] = 0.0;
  cvs->struct_odometry->xsiRpoint[2] = 0.0;

}



/* This function computes the wheels velocities x_dot,y_dot and theta_dot in the robot space
 *
 * param[in] : cvs controller main structure
 * out : [x_dot,y_dot,theta_dot]
 */
void xsiRWheels(CtrlStruct *cvs, int unit) // 1 = mm & 0 = meter
{
    double radius = 0.03;
    double l = 0.1500; // to be modified 
    // phi_1 = roue droite, phi_2 = roue gauche
    cvs->struct_odometry->xsiRpoint[0] = (radius*cvs->inputs->r_wheel_speed)*0.5 + (radius*cvs->inputs->l_wheel_speed)*0.5;
    cvs->struct_odometry->xsiRpoint[2] = (radius*cvs->inputs->r_wheel_speed)/(2*l) - (radius*cvs->inputs->l_wheel_speed)/(2*l);
    if (unit ==1)
    {
      cvs->struct_odometry->xsiRpoint[0] = cvs->struct_odometry->xsiRpoint[0]*1000;  
    }
}


/* This function computes the position of the robot (=odometry)
 *
 * param[in] : cvs controller main structure, table with speeds of the wheels
 * out : [xr,yr]
 */
void computePosition(CtrlStruct *cvs)
{

    double v = cvs->struct_odometry->xsiRpoint[0];
    double w = cvs->struct_odometry->xsiRpoint[2];

    double delta_t = 0.04;

    double theta = w*delta_t + cvs->struct_odometry->theta_t;
    if(theta>PI)
    {
      theta=theta-2*PI;
    }
    if(theta<-PI)
    {
        theta=theta+2*PI;
    }
    cvs->struct_odometry->theta_t = theta;

    double x = v*cos(theta)*delta_t + cvs->struct_odometry->x_t;
    double y = v*sin(theta)*delta_t + cvs->struct_odometry->y_t;

    cvs->struct_odometry->x_t = x;
    cvs->struct_odometry->y_t = y;

}

// To do : mettre delta_t dans la structure!

/* This function displays some useful informations about the wheels
 *
 * param[in] : cvs controller main structure
 *
 */
void displayOdometry(CtrlStruct *cvs) {


    CtrlIn *ivs;
    CtrlOut *ovs;
    StructOdometry *odometry;

    ivs = cvs->inputs;
    ovs = cvs->outputs;
    odometry = cvs->struct_odometry;


    printf("Position x = %f \n",odometry->x_t);
    printf("Position y = %f \n",odometry->y_t);
    printf("Position theta = %f\n",odometry->theta_t);

}
//NAMESPACE_CLOSE();
