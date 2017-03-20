//
//  Keyboard_gr1.hpp
//  
//
//  Created by William Chermanne on 24/02/17.
//
//

#include <stdio.h>
#ifndef Keyboard_gr1_hpp
#define Keyboard_gr1_hpp


#include "ctrl_io.h"
#include "namespace_ctrl.h"
// #include "my_own_headers.h" // adapt it with your headers

NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number

void keyboardContr(CtrlStruct *cvs);




NAMESPACE_CLOSE();


#endif 
