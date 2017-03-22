//
//  FSM_gr1.cpp
//
//
//  Created by William Chermanne on 6/03/17.
//
//
#include "MyIncludes_gr1.h"
#define NBR_STATES 3
#define NBR_TARGETS 7

NAMESPACE_INIT(ctrlGr1);

/* This function initializes the StructFSMstructure
 *
 * param[in] : cvs controller main structure
 */

void StructFSM_init(CtrlStruct *cvs)
{
  printf(RED "I'm initializing the FSM \n" RESET);
  cvs->struct_fsm->indexNextTarget=0;
  cvs->struct_fsm->TargetArray = (Target**) malloc(NBR_TARGETS*sizeof(Target));
  cvs->struct_fsm->robot_state=BEGINNING;
  cvs->struct_fsm->isGenerated=0;
  printf(GRN "I'm DONE initializing the FSM \n" RESET);

for (int i = 0; i < NBR_TARGETS; i++)
{
  cvs->struct_fsm->TargetArray[i] = (Target*) malloc(sizeof(Target));
  cvs->struct_fsm->TargetArray[i]->isTaken=0;
  cvs->struct_fsm->TargetArray[i]->indexTarget=i;
}

    cvs->struct_fsm->TargetArray[0]->x=-400;
    cvs->struct_fsm->TargetArray[0]->y=-1250;
    cvs->struct_fsm->TargetArray[0]->score=1;

    cvs->struct_fsm->TargetArray[1]->x=800;
    cvs->struct_fsm->TargetArray[1]->y=-1300;
    cvs->struct_fsm->TargetArray[1]->score=2;

    cvs->struct_fsm->TargetArray[2]->x=800;
    cvs->struct_fsm->TargetArray[2]->y=-550;
    cvs->struct_fsm->TargetArray[2]->score=1;

    cvs->struct_fsm->TargetArray[3]->x=0;
    cvs->struct_fsm->TargetArray[3]->y=0;
    cvs->struct_fsm->TargetArray[3]->score=3;

    cvs->struct_fsm->TargetArray[4]->x=800;
    cvs->struct_fsm->TargetArray[4]->y=550;
    cvs->struct_fsm->TargetArray[4]->score=1;

    cvs->struct_fsm->TargetArray[5]->x=-400;
    cvs->struct_fsm->TargetArray[5]->y=1250;
    cvs->struct_fsm->TargetArray[5]->score=1;

    cvs->struct_fsm->TargetArray[6]->x=800;
    cvs->struct_fsm->TargetArray[6]->y=1300;
    cvs->struct_fsm->TargetArray[6]->score=2;

}


void matchFSM(CtrlStruct *cvs)
{

  CtrlIn *ivs;
  CtrlOut *ovs;
  StructOdometry *odometry;

  ivs = cvs->inputs;
  ovs = cvs->outputs;
  odometry = cvs->struct_odometry;

  double xStart,yStart;
  double xTarget,yTarget;
  int StartNode;
  int TargetNode;
  int i=1;
  double *KpKi;
  double r_wheel_speed;
  double l_wheel_speed;
  double *tempxsirpoint;
  double Kp,Ki;
  bool found;
  int maxScore=0;
  int indexNextTarget;
  double *command=cvs->struct_control->command;
  double error;

  //enum {BEGINNING,WAIT , FIRST_TARGET}; // replace 'STATE_X' by a more explicit name

  switch (cvs->struct_fsm->robot_state)
  {

    ////////////////////////////  NEW STATE   ////////////////////////////
    ////////////////////////////  NEW STATE   ////////////////////////////
    ////////////////////////////  NEW STATE   ////////////////////////////

    case BEGINNING:

    printf(RED "I'm in the BEGINNING state\n" RESET);
    Astar_init(cvs);
    StructControl_init(cvs);
    RefreshPath(cvs->struct_path_planning->astar);
    cvs->struct_fsm->robot_state =SET_OBJECTIVE;

    break;

    ////////////////////////////  NEW STATE   ////////////////////////////
    ////////////////////////////  NEW STATE   ////////////////////////////
    ////////////////////////////  NEW STATE   ////////////////////////////
    case SET_OBJECTIVE:
    for (int i = 0; i < NBR_TARGETS; i++)
    {
      if(cvs->struct_fsm->TargetArray[i]->score>maxScore && cvs->struct_fsm->TargetArray[i]->isTaken==0)
      {
        maxScore=cvs->struct_fsm->TargetArray[i]->score;
        indexNextTarget=cvs->struct_fsm->TargetArray[i]->indexTarget;
        printf("Index Next Target: %d\n",indexNextTarget);
      }
      printf("Index %d (maxscore %d)\n",indexNextTarget,maxScore);
    }
    cvs->struct_fsm->indexNextTarget=indexNextTarget;

    // Nextstate = Generate_path
    cvs->struct_fsm->robot_state =GENERATE_PATH;

    break;

////////////////////////////  NEW STATE   ////////////////////////////
////////////////////////////  NEW STATE   ////////////////////////////
////////////////////////////  NEW STATE   ////////////////////////////


    case GENERATE_PATH:

    printf(GRN "I'm in the GENERATE_PATH state\n" RESET);
    indexNextTarget=cvs->struct_fsm->indexNextTarget;
    xTarget=cvs->struct_fsm->TargetArray[indexNextTarget]->x;
    yTarget=cvs->struct_fsm->TargetArray[indexNextTarget]->y;

    if(cvs->struct_fsm->isGenerated==0)
    {
      xStart=cvs->struct_odometry->x_t;
      yStart=cvs->struct_odometry->y_t;
      printf(CYN "Position x:%f\n",cvs->struct_odometry->x_t);
      printf(CYN "Position y:%f\n",cvs->struct_odometry->y_t);
      printf(CYN "Angle:%f\n",(cvs->struct_odometry->theta_t)*180/3.14);



      ////// Create the path //////
      StartNode=FindNearestNode(cvs->struct_path_planning->astar,xStart,yStart);
      TargetNode= FindNearestNode(cvs->struct_path_planning->astar,xTarget,yTarget);
      Create_path(cvs->struct_path_planning->astar, StartNode,TargetNode);
      cvs->struct_fsm->isGenerated=1;
    }
      // Nextstate = Follow_path
      cvs->struct_fsm->robot_state =FOLLOW_PATH;

    break;


    ////////////////////////////  NEW STATE   ////////////////////////////
    ////////////////////////////  NEW STATE   ////////////////////////////
    ////////////////////////////  NEW STATE   ////////////////////////////

    case FOLLOW_PATH:


    //////////////////// ODOMETRY ////////////////////
    tempxsirpoint = (double *)malloc(sizeof(double)* 3);
    xsiRWheels(cvs,tempxsirpoint);
    computePosition(cvs,tempxsirpoint);


    CtrlIn *ivs;
    CtrlOut *ovs;
    StructOdometry *odometry;

    ivs = cvs->inputs;
    ovs = cvs->outputs;
    odometry = cvs->struct_odometry;
    command=cvs->struct_control->command;

    indexNextTarget=cvs->struct_fsm->indexNextTarget;
    xTarget=cvs->struct_fsm->TargetArray[indexNextTarget]->x;
    yTarget=cvs->struct_fsm->TargetArray[indexNextTarget]->y;

    ovs->tower_command = 10.0;

    KpKi = Kp_Ki_Computation(0.05,0.01);

    FromHighToMiddleLevel(cvs,command);
    MiddleLevelController(command[0]/1000,command[1],command);
    LowLevelController(cvs,command, KpKi[0], KpKi[1],command);

    ovs->wheel_commands[R_ID] = command[0];
    ovs->wheel_commands[L_ID] = command[1];

    free(tempxsirpoint);
    free(KpKi);




    error=EuclidianDistance(xTarget,yTarget,cvs->struct_odometry->x_t,cvs->struct_odometry->y_t);

    printf(RED "indexNextTarget:%d\n",indexNextTarget);
    printf(RED "Position xTARGET:%f\n",xTarget);
    printf(RED "Position yTARGET:%f\n",yTarget);
    printf(CYN"error: %f\n"RESET,error);
    if (error <100.0)
    {
      cvs->struct_fsm->TargetArray[indexNextTarget]->isTaken=1;
      cvs->struct_fsm->isGenerated=0;
      cvs->struct_fsm->robot_state =BEGINNING;
    }
    break;

    ////////////////////////////  NEW STATE   ////////////////////////////
    ////////////////////////////  NEW STATE   ////////////////////////////
    ////////////////////////////  NEW STATE   ////////////////////////////

    default:
    printf("Error: unknown state : %d !\n", cvs->struct_fsm->robot_state);
    exit(EXIT_FAILURE);
  }

}


void StructFSM_free(CtrlStruct *cvs)
{
  for (int i = 0; i < NBR_TARGETS; i++)
  {
    free(cvs->struct_fsm->TargetArray[i]);
  }
  free(cvs->struct_fsm->TargetArray);
}
NAMESPACE_CLOSE();
