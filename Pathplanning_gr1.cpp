//
//  Pathplanning_gr1.cpp
//
//
//  Created by William Chermanne on 28/02/17.
//
//

#include "ctrl_main_gr1.h"
#include "namespace_ctrl.h"
#include "Pathplanning_gr1.hpp"
#include "CtrlStruct_gr1.h"
#include <math.h>
#include <stdlib.h>
#include "user_realtime.h"
#include "Astar_gr1.hpp"

NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number

/* This function initializes the StructPathPlanning structure
 *
 * param[in] : cvs controller main structure
 */

void StructPathPlanning_init(CtrlStruct *cvs)
{
  //Astar_init(cvs);
}
/* This function computes the forward and rotational speed to apply to reach the goal coordinates
 *
 * param[in] : cvs controller main structure, table with speeds of the wheels
 * out : [xr,yr]
 */
double HighLevelController(CtrlStruct *cvs, double goalx, double goaly, double precisionRadius )
{

    return 0;
}



NAMESPACE_CLOSE();
