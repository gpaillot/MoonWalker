/*!
 * \file CtrlStruct_gr1.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR1_H_
#define _CTRL_STRUCT_GR1_H_

#include "ctrl_io.h"
//#include "namespace_ctrl.h"
#include <stdlib.h>

//NAMESPACE_INIT(ctrlGr1);



typedef struct StructTower
{
    int counter;///< simple variable
    int counterDist;
    double *tabDistance;
    double *tabFixed; ///< 1D tabular of double
    double dist;
    double previousDistance;
    int previous_rising_index;
    int previous_falling_index;
} StructTower;

// StructWheels
typedef struct StructWheels
{
    int counter;///< simple variable
    double *prev_distance;
    double *prev_speed; ///< 1D tabular of double for previous speed of both wheels
    double x_t;
    double y_t;
    double theta_t;

} StructWheels;

// Controller structure
typedef struct StructControl
{
  double Kp;          // Coefficient for the P controller
  double Ki;          // Coefficient for I controller
  double *sum_error;  // sum of all errors
  double Kt;          // Anti-windup constant
  double Tsample;     // sampling period
  double *currentError;        // speed reference - measured speed
  double *previousError;       // error from previous iteration
  double *previousCommand;     // command from previous iteration
  double *previousCommandLtd;  // limited command from previous iteration

}StructControl;



typedef struct StructPathPlanning
{


} StructPathPlanning;


/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs


  // Created structures
  StructTower *struct_tower;
  StructWheels *struct_wheels;
  StructControl *struct_control;
  StructPathPlanning *struct_path_planning;

} CtrlStruct;


// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);

//NAMESPACE_CLOSE();

#endif
