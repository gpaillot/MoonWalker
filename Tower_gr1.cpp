//
//  Tower_gr1.cpp
//
//
//  Created by William Chermanne on 20/02/17.
//
//


/*!
 * \file Tower_gr1.cc
 * \brief File to use the tower
 */

#include "ctrl_main_gr1.h"
#include "namespace_ctrl.h"
#include "Tower_gr1.hpp"
#include "CtrlStruct_gr1.h"
#include <math.h>
#include <stdlib.h>
#include "user_realtime.h"
#include "Controllers_gr1.hpp"
#include "Pathplanning_gr1.hpp"
#include "UsefulFunctions_gr1.hpp"


#define PI 3.14159265358979323846

NAMESPACE_INIT(ctrlGr1);


void StructTower_init(CtrlStruct *cvs)
{
  //////////////////////   Structure Tower //////////////////////

  // Allocations
  cvs->struct_tower->tabFixed = (double*) malloc(3*sizeof(double));
  cvs->struct_tower->tabDistance = (double*) malloc(10*sizeof(double)); // Tabular of doubles of size 10

  cvs->struct_tower->counter=0;
  cvs->struct_tower->previous_rising_index=0;
  cvs->struct_tower->counterDist=0;
  cvs->struct_tower->dist=0;
  cvs->struct_tower->previousDistance=0;

}


/* This function returns the angle between our robot and the opponent robot
 *
 * param[in] : cvs controller main structure
 *
 * return[out] : angle opponent
 */
double angle_opponent(CtrlStruct *cvs) {


    ///////////// Classical /////////////////

    CtrlIn *ivs;
    ivs = cvs->inputs;

    ///////////// Call variables /////////////////

    // General
    double tower_pos= ivs->tower_pos;

    // For fixed beacons
    double *last_rising_fixed = ivs->last_rising_fixed;
    double *last_falling_fixed = ivs->last_falling_fixed;
    int rising_index_fixed = ivs->rising_index_fixed;
    int falling_index_fixed = ivs->falling_index_fixed;
    int nb_rising_fixed=ivs->nb_rising_fixed;

    // Opponent beacons
    double *last_rising = ivs->last_rising;
    double *last_falling = ivs->last_falling;
    int rising_index = ivs->rising_index;
    int falling_index = ivs->falling_index;
    int nb_opponents = ivs->nb_opponents;



    ///////////// Calculation of the angles /////////////////

    // Angles
    double rising_edge_angle = (last_rising[rising_index]); // To pick the right angle thanks to the index
    double falling_edge_angle = (last_falling[falling_index]);

    double angle_opponent = (rising_edge_angle + falling_edge_angle)/2;

    return angle_opponent;

}

/* This function returns the distance between our robot and the opponent's robot
 *
 * param[in] : cvs controller main structure
 *
 * return[out] : distance between our robot and the opponent's robot (temporal mean of the last 10 distance calculated)
 */

double distance_opponent(CtrlStruct *cvs) {

    ///////////// Classical /////////////////

    CtrlIn *ivs;
    ivs = cvs->inputs;

    ///////////// Call variables /////////////////


    // From inputs

    double *last_rising = ivs->last_rising;
    double *last_falling = ivs->last_falling;
    int rising_index = ivs->rising_index;
    int falling_index = ivs->falling_index;
    double rising_edge_angle = (last_rising[rising_index]);
    double falling_edge_angle = (last_falling[falling_index]);

    // From StructTower

    int counterDist = cvs->struct_tower->counterDist;
    double *tabDistance = cvs->struct_tower->tabDistance;
    double previousDistance = cvs->struct_tower->previousDistance;

    // Data
    int d_beacon=8;
    int r_beacon=4;

    ///////////// Calculations /////////////////

    double arg1=sin((rising_edge_angle-falling_edge_angle)/2);
    double arg2= r_beacon/arg1;
    double distance=arg2;

    if(distance<0) // Just to make it positive (function abs generates a warning and it irritates me :-) )
    {
        distance = - distance;
    }

    //////// From here, just to take the temporal mean of the last 10 distances calculated  ////////

    tabDistance[counterDist]=distance;
    cvs->struct_tower->dist=distance;
    if(cvs->struct_tower->counterDist==9)
    {
        cvs->struct_tower->counterDist=0;
    }
    else if( cvs->struct_tower->previousDistance != distance)
    {
        cvs->struct_tower->counterDist++;
        cvs->struct_tower->previousDistance=distance;
    }

    double moyenne_temp = (tabDistance[0]+tabDistance[1]+tabDistance[2]+tabDistance[3]+tabDistance[4]+tabDistance[5]+tabDistance[6]+tabDistance[7]+tabDistance[8]+tabDistance[9])/10;

    return moyenne_temp;
}


/* This function returns the angles of the fixed beacons (in radians!)
 *
 * param[in] : cvs controller main structure
 *
 * return[out] : Tabular three angles of the fixed beacons.
 *                  Entry 0 = beacon located at (-1062,+1562)
 *                  Entry 1 = beacon located at (0,-1562)
 *                  Entry 2 = beacon located at (+1062,+1562)
 *
 */

double *anglesTable_fixed(CtrlStruct *cvs) {

    ///////////// Classical /////////////////

    CtrlIn *ivs;
    ivs = cvs->inputs;


    ///////////// Call variables /////////////////

    // From StructTower
    int counter = cvs->struct_tower->counter;
    double *tab = cvs->struct_tower->tabFixed;

    // From inputs
    double tower_pos= ivs->tower_pos;
    double *last_rising_fixed = ivs->last_rising_fixed;
    double *last_falling_fixed = ivs->last_falling_fixed;
    int rising_index_fixed = ivs->rising_index_fixed;
    int falling_index_fixed = ivs->falling_index_fixed;
    int nb_rising_fixed=ivs->nb_rising_fixed;
    int nb_falling_fixed=ivs->nb_falling_fixed;

    ///////////// Computations /////////////////

    // Each time this function is called, we calculate an angle
    double rising_edge_angle_fixed = (last_rising_fixed[rising_index_fixed]);
    double falling_edge_angle_fixed = (last_falling_fixed[falling_index_fixed]);
    double angle_beacon_fixed = (rising_edge_angle_fixed + falling_edge_angle_fixed)/2;

    // We then verify some stuff
    if((rising_index_fixed==falling_index_fixed && nb_rising_fixed>0 && nb_falling_fixed>0) && (cvs->struct_tower->previous_rising_index != rising_index_fixed))
    {
        tab[counter]=angle_beacon_fixed; // Save the angle at the correct place in the tabular of the struct_tower. We must then increment the counter to say that a beacon was saved! The counter goes from 0 to 2.

        if(counter==2) // If we have saved the angles of the 3 beacons, set the counter to 0
        {
            cvs->struct_tower->counter=0;
        }
        else // Increment the counter if we haven't set the angles of the 3 beacons
        {
            cvs->struct_tower->counter++;
        }
        cvs->struct_tower->previous_rising_index = rising_index_fixed; // previous=current
    }

    return tab; // Returns a table with the angles of the fixed beacons
}


/* This function allows an easy call to the cotangent function, unavailable in math.h
 *
 * param[in] : angle a
 * return[out] : cotangent of a
 */

double cotan(double a)
{
    return cos(a)/sin(a);
}


/* This function returns the position of the robot using the position of 3 beacons and their angle with respect to the robot.
 * This code was found courtesy of the work of Vincent Pierlot and Marc Van Droogenbroeck, Montefiore Research Institute, University of Liège, Belgium. It is available online at : http://www.telecom.ulg.ac.be/triangulation/

 * The method implemented here is an adaptation of their ToTal algorithm.

 * param[in] : angles w/ respect to the robot of beacons 1, 2 and 3, and beacon "i" coordinates : xi and yi.
 * return[out] : coordinates array
 */

double *triangulation(double alpha1, double alpha2, double alpha3,double x1, double y1, double x2, double y2, double x3, double y3)
{
    double *coordinates;
    coordinates = (double*) malloc(2*sizeof(double));

    double x;
    double y;

    double cot_max= 1000000000.0;

    double cot_12 = cotan(alpha2 - alpha1);
    double cot_23 = cotan(alpha3 - alpha2);

    cot_12 = adjust_value_to_bounds(cot_12, cot_max);
    cot_23 = adjust_value_to_bounds(cot_23, cot_max);

    double cot_31 = ( 1.0 - cot_12 * cot_23 ) / ( cot_12 + cot_23 ) ;
    cot_31 = adjust_value_to_bounds( cot_31 , cot_max) ;

    double x1_ = x1 - x2 , y1_ = y1 - y2 , x3_ = x3 - x2 , y3_ = y3 - y2 ;

    double c12x = x1_ + cot_12 * y1_ ;
    double c12y = y1_ - cot_12 * x1_ ;

    double c23x = x3_ - cot_23 * y3_ ;
    double c23y = y3_ + cot_23 * x3_ ;

    double c31x = (x3_ + x1_) + cot_31 * (y3_ - y1_) ;
    double c31y = (y3_ + y1_) - cot_31 * (x3_ - x1_) ;

    double k31 = (x3_ * x1_) + (y3_ * y1_) + cot_31 * ((y3_ * x1_) - (x3_ * y1_)) ;

    double D = (c12x - c23x) * (c23y - c31y) - (c23x - c31x) * (c12y - c23y) ;
    double invD = 1.0 / D ;
    double K = k31 * invD ;

    x = K * (c12y - c23y) + x2 ;
    y = K * (c23x - c12x) + y2 ;

    coordinates[0]=x;
    coordinates[1]=y;

    return coordinates;
}

/* This function adjusts the value of the cotangent because it could reach infinity
 *
 * param[in] : cotangent
 *
 * param[out] : adjusted cotangent
 */

double adjust_value_to_bounds( double cot , double cot_max )
{
    if (cot>cot_max){
        cot=cot_max;
    }
    else if (cot < -cot_max){
        cot=-cot_max;
    }

    return cot;
}


/* This function displays some useful informations about the tower
 *
 * param[in] : cvs controller main structure
 *
 */

void tower_display(CtrlStruct *cvs) {


    CtrlIn *ivs;
    ivs = cvs->inputs;

    ////// Interesting inputs //////

    // General inputs
    double tower_pos= ivs->tower_pos;
    int nb_opponents = ivs->nb_opponents;
    double t = ivs->t;

    // Fixed Beacons locations: 2 different sets depending on the IDs of the robot!

    double x1,y1,x2,y2,x3,y3;
    if(cvs->inputs->robot_id<2)
    {
        x1=0;
        y1=-1562;

        x2=1062;
        y2=1562;

        x3=-1062;
        y3=1562;

    }
    else
    {
        x1=0;
        y1=1562;

        x2=-1062;
        y2=-1562;

        x3=1062;
        y3=-1562;

    }
    // Triangulation

    // Fixed beacons angles
    double *fixedBeacons = anglesTable_fixed(cvs);

    // alpha 1 = beacon located at (0,-1562)
    // alpha 2 = beacon located at (1062,1562)
    // alpha 3 = beacon located at (-1062,1562)

    double alpha1=(fixedBeacons[1]);
    double alpha2=(fixedBeacons[2]);
    double alpha3=(fixedBeacons[0]);

    // Just to have positive angles

    if(alpha1<0)
    {
        alpha1=(2*PI)+alpha1;
    }
    if(alpha2<0)
    {
        alpha2=(2*PI)+alpha2;
    }
    if(alpha3<0)
    {
        alpha3=(2*PI)+alpha3;
    }




    // Fixed beacons
    double *last_rising_fixed = ivs->last_rising_fixed;
    double *last_falling_fixed = ivs->last_falling_fixed;
    int rising_index_fixed = ivs->rising_index_fixed;
    int falling_index_fixed = ivs->falling_index_fixed;
    int nb_rising_fixed=ivs->nb_rising_fixed;

    // Opponent beacon
    double *last_rising = ivs->last_rising;
    double *last_falling = ivs->last_falling;
    int rising_index = ivs->rising_index;
    int falling_index = ivs->falling_index;
    int counterDist = cvs->struct_tower->counterDist;



    ///////////// Computations /////////////////

    // Opponent
    double angle_opp= angle_opponent(cvs);
    double dist_moyenne=distance_opponent(cvs);

    // Computation of the coordinates of the robot thanks to triangulation
    double *coord;
    coord = (double*) malloc(2*sizeof(double));
    coord=triangulation(alpha1,alpha2,alpha3,x1,y1,x2,y2,x3,y3);


    double euclidian= EuclidianDistance(0,0,coord[0]/1000,coord[1]/1000);
    ////// Some printf's //////

    printf("***********   TIME   ***********    \n");
    printf("Time: %f \n",t);

    printf("\n");

    printf("***********   GENERAL   ***********    \n");
    printf("Tower position in degrees: %f \n",(tower_pos*180)/PI);

    printf("\n");


    printf("***********   OPPONENT   ***********    \n");
    printf("Number of opponents: %d \n",nb_opponents);
    printf("Index of moving beacons : %d\n",rising_index);
    printf("Angle of the opponent's robot: %f \n", (angle_opp)*180/PI);
    printf("Distance counter: %d \n", counterDist);
    printf("Distance: %f \n", cvs->struct_tower->dist);
    printf("Distance moyennée: %f \n", dist_moyenne);

    printf("\n");

    printf("***********   FIXED BEACONS   ***********    \n");
    printf("Number of rising fixed edges: %d \n",nb_rising_fixed);
    printf("Number of falling fixed edges: %d \n",nb_rising_fixed);
    printf("Index of rising fixed edges : %d\n",rising_index_fixed);
    printf("Index of rising fixed edges : %d\n",falling_index_fixed);
    printf("Counter : %d \n",cvs->struct_tower->counter);
    printf("Angle of fixed beacon 1 (=alpha 3): %f\n",(fixedBeacons[0])*180/PI);
    printf("Angle of fixed beacon 2 (=alpha 1): %f\n",(fixedBeacons[1])*180/PI);
    printf("Angle of fixed beacon 3 (=alpha 2): %f\n",(fixedBeacons[2])*180/PI);

    printf("\n");

    printf("***********   TRIANGULATION   ***********    \n");

    printf("Angle alpha 1 : %f\n",(alpha1)*180/PI);
    printf("Angle alpha 2: %f\n",(alpha2)*180/PI);
    printf("Angle alpha 3: %f\n",(alpha3)*180/PI);

    printf("Robot coordinate Xr [m]: %f\n",(coord[0])/1000);
    printf("Robot coordinate Yr [m]: %f\n",(coord[1])/1000);
    printf("Euclidian Distance [m]: %f\n", euclidian);
}


NAMESPACE_CLOSE();
