//
//  Keyboard_gr1.cpp
//  
//
//  Created by William Chermanne on 24/02/17.
//
//



#include "ctrl_main_gr1.h"
#include "namespace_ctrl.h"
#include "Keyboard_gr1.hpp"
#include "CtrlStruct_gr1.h"
#include "ctrl_io.h"
#include <math.h>
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr1);

void keyboardContr(CtrlStruct *cvs)
{
    CtrlIn *ivs;
    CtrlOut *ovs;
    
    ivs = cvs->inputs;
    ovs = cvs->outputs;
    
    if(ivs->keyboard_arrow[0][0]>0)
    {
        ovs->wheel_commands[R_ID] = 50;
        ovs->wheel_commands[L_ID] = 50;
    }
    
    if(ivs->keyboard_arrow[0][0]>0)
    {
        ovs->wheel_commands[R_ID] = 40;
        ovs->wheel_commands[L_ID] = 40;
        
    }
    if(ivs->keyboard_arrow[0][0]<0)
    {
        ovs->wheel_commands[R_ID] = -40;
        ovs->wheel_commands[L_ID] = -40;
        
    }
    if(ivs->keyboard_arrow[0][1]>0)
    {
        ovs->wheel_commands[R_ID] = 40;
        ovs->wheel_commands[L_ID] = 0;
        
    }
    if(ivs->keyboard_arrow[0][1]<0)
    {
        ovs->wheel_commands[R_ID] = 0;
        ovs->wheel_commands[L_ID] = 40;
        
    }
    
    if(ivs->keyboard_key[0]==1)
    {
        ovs->wheel_commands[R_ID] = 0;
        ovs->wheel_commands[L_ID] = 0;
        
    }
}

NAMESPACE_CLOSE();
