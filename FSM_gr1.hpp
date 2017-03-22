//
//  FSM_gr1.hpp
//
//
//  Created by William Chermanne on 6/03/17.
//
//

#ifndef FSM_gr1_hpp
#define FSM_gr1_hpp

#include <stdio.h>
NAMESPACE_INIT(ctrlGr1);
void StructFSM_init(CtrlStruct *cvs);
void matchFSM(CtrlStruct *cvs);
void StructFSM_free(CtrlStruct *cvs);
NAMESPACE_CLOSE();

#endif /* FSM_gr1_hpp */
