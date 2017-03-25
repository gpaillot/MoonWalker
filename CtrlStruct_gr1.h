/*!
 * \file CtrlStruct_gr1.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR1_H_
#define _CTRL_STRUCT_GR1_H_

#include "ctrl_io.h"
//#include "namespace_ctrl.h"
#include <stdlib.h>
//#include "Astar_struct_gr1.h"

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
typedef struct StructOdometry
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
  double Kp; // Coefficient for the P controller
  double Ki; // Coefficient for I controller
  double sum_error[2]; // sum of all errors
  double currentError[2];
  double previousError[2];
  double previousCommand[2];
  double previousCommandLtd[2];
  double errDist;
  double errAngle;
  int counterNode;
  double command[2];
  double Speed_ref[2];
  double Kt;          // Anti-windup constant
  double Tsample;     // sampling period

}StructControl;



typedef struct StructPathPlanning
{
  //Astar *astar;
} StructPathPlanning;

typedef struct StructFSM
{
  double test;
} StructFSM;


/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs
	CtrlOut *py_outputs; ///< python controller outputs

  // Created structures
  StructTower *struct_tower;
  StructOdometry *struct_odometry;
  StructControl *struct_control;
  StructPathPlanning *struct_path_planning;
  StructFSM *struct_fsm;


} CtrlStruct;


// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);

//NAMESPACE_CLOSE();

#endif
