


#include "CtrlStruct_gr1.h"
//#include "namespace_ctrl.h"
/*#include "Tower_gr1.hpp"
#include "Odometry_gr1.hpp"*/
#include "Controllers_gr1.hpp"
/*#include "Pathplanning_gr1.hpp"
#include "Astar_gr1.hpp"
#include "FSM_gr1.hpp"*/


/*! \brief initialize the controller structure
 *
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 */
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs)
{
  int i;
  CtrlStruct *cvs;
  cvs->struct_tower= (StructTower*) malloc(sizeof(StructTower));   // Allocate space
  cvs->struct_odometry = (StructOdometry*) malloc(sizeof(StructOdometry));
  cvs->struct_control = (StructControl *) malloc(sizeof(StructControl));
  cvs->struct_path_planning= (StructPathPlanning *) malloc(sizeof(StructPathPlanning));
  cvs->struct_fsm= (StructFSM *) malloc(sizeof(StructFSM));

  cvs->inputs  = inputs;
  cvs->outputs = outputs;
////// Here, we can implement new structures! Do not forget to add their definition in CtrlStruct_gr1.h! Here, only the initialization //////
// To initialize all the structures

//  StructTower_init(cvs);
  //StructPathPlanning_init(cvs);
  StructControl_init(cvs);
  StructOdometry_init(cvs);
  //StructFSM_init(cvs);
return cvs;
}

/*! \brief release controller main structure memory
 *
 * \param[in] cvs controller main structure
 */

 void free_CtrlStruct(CtrlStruct *cvs)
 {
     free(cvs->inputs);
     free(cvs->outputs);
     free(cvs->py_outputs);

     // Structure tower
     free(cvs->struct_tower->tabFixed);
     free(cvs->struct_tower->tabDistance);
     free(cvs->struct_tower);
     // struct. for wheels
     free(cvs->struct_odometry->prev_speed);
     free(cvs->struct_odometry->prev_distance);
     free(cvs->struct_odometry);
     // struct. for control
     free(cvs->struct_control->sum_error);
     free(cvs->struct_control);
     //structure path planning
    // Astar_free(cvs->struct_path_planning->astar);
     free(cvs->struct_path_planning);
     free(cvs);

 }
