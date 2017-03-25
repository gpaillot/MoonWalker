/*!
 * \file CtrlStruct_gr1.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR1_H_
#define _CTRL_STRUCT_GR1_H_

#include "ctrl_io.h"
//#include "namespace_ctrl.h"
#include <stdlib.h>
#include "Astar_struct_gr1.h"

NAMESPACE_INIT(ctrlGr1);

enum RobotState
{
    BEGINNING,
    GENERATE_PATH,
    SET_OBJECTIVE,
    FOLLOW_PATH,
    GO_BACK_BASIS
};

typedef struct StructTower
{
    int counter;///< simple variable
    int counterDist;
    double tabDistance[10];
    double tabFixed[3]; ///< 1D tabular of double
    double dist;
    double previousDistance;
    int previous_rising_index;
    int previous_falling_index;
} StructTower;

// StructWheels
typedef struct StructOdometry
{
    int counter;///< simple variable
    double prev_distance[2];
    double prev_speed[2]; ///< 1D tabular of double for previous speed of both wheels
    double x_t;
    double y_t;
    double theta_t;
    bool moonwalker;

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
  double Kt;          // Anti-windup constant
  double Tsample;     // sampling period

}StructControl;



typedef struct StructPathPlanning
{
  Astar *astar;
} StructPathPlanning;


typedef struct Target
{
  double x;
  double y;
  bool isTaken;
  int indexTarget;
  int score;
} Target;


typedef struct StructFSM
{
  RobotState robot_state;
  Target **TargetArray;
  bool isGenerated;
  int indexNextTarget;
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
