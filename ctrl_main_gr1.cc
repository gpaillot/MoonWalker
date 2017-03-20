/*!
 * \file ctrl_main_gr1.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

 #include "MyIncludes_gr1.h"

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define BLU   "\x1B[34m"
#define RESET "\x1B[0m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"



//////// NOTE : QUAND ON CREE UN NOUVEAU FICHIER , NE PAS OUBLIE DE TOUT COPIER DANS LE HAUT DES FICHIERS .h et .cc, DE FAIRE TOUS LES INCLUDE ET PUIS DE FAIRE CMAKE .. SINON CA FOIRE "

NAMESPACE_INIT(ctrlGr1);

/*! \brief initialize controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
  printf("Initializing the controller");

  Astar_init(cvs);
  double xStart,yStart;
  int StartNode=FindNearestNode(cvs->struct_path_planning->astar,-850,-1150);
  int TargetNode= FindNearestNode(cvs->struct_path_planning->astar,800,1300);

  printf(RED "StartNode index: %d \n" RESET,StartNode);
  printf(RED "TargetNode index: %d \n" RESET,TargetNode);

  printf("Controller initialized\n");

  Create_path(cvs->struct_path_planning->astar, StartNode,TargetNode); // Dernière fonction que j'appelle pour le high level
  SelectNodesPath(cvs->struct_path_planning->astar,cvs->struct_path_planning->astar->path);
  CreatePathOnlyTurns(cvs->struct_path_planning->astar);

  createTxtFile("Nodes.txt",cvs->struct_path_planning->astar);

  printf("%s %s\n", __DATE__, __TIME__);

  // Apd d'ici, on a un path
}


/*! \brief controller loop (called every timestep)
 *
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{

  double Kp,Ki;
  //////////////////// ODOMETRY ////////////////////
  double *tempxsirpoint;
  tempxsirpoint = (double *)malloc(sizeof(double)* 3);
  tempxsirpoint = xsiRWheels(cvs);
  computePosition(cvs,tempxsirpoint);

  double *command=cvs->struct_control->command;

  int counter;
  CtrlIn *ivs;
  CtrlOut *ovs;
  StructOdometry *odometry;

  ivs = cvs->inputs;
  ovs = cvs->outputs;
  odometry = cvs->struct_odometry;


  double r_wheel_speed = ivs->r_wheel_speed;
  double l_wheel_speed = ivs->l_wheel_speed;


  ovs->tower_command = 15.0;


  double *KpKi = Kp_Ki_Computation(0.05,0.0035);

  if(ivs->t >-5.0)
  {
    printf(RED "Entering the loop\n" RESET );
    //double *HighToMiddle= FromHighToMiddleLevel(cvs); //
    FromHighToMiddleLevel(cvs,command);
    printf(GRN "From High To middle passed\n" RESET );

    //double *MiddleToLow=MiddleLevelController(HighToMiddle[0]/1000,HighToMiddle[1]);
    //command = MiddleLevelController(command[0]/1000,command[1]);
    MiddleLevelController(command[0]/1000,command[1],command); // Vient modifier command


    //double *WheelCommand = LowLevelController(cvs,MiddleToLow, KpKi[0], KpKi[1]);
    LowLevelController(cvs,command, KpKi[0], KpKi[1],command);// Vient modifier command
    //keyboardContr(cvs);

  //  ovs->wheel_commands[R_ID] = WheelCommand[0];
  //  ovs->wheel_commands[L_ID] = WheelCommand[1];

    ovs->wheel_commands[R_ID] = command[0];
    ovs->wheel_commands[L_ID] = command[1];

    //free(HighToMiddle);
    free(tempxsirpoint);
    free(KpKi);
  }

}

/*! \brief last controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{
  Astar_free(cvs->struct_path_planning->astar);
}

NAMESPACE_CLOSE();
