//
//  Obstacles_gr1.hpp
//
//
//  Created by William Chermanne on 13/03/17.
//
//
#include <stdio.h>

#ifndef Obstacles_gr1_hpp
#define Obstacles_gr1_hpp

#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr1); // where X should be replaced by your group number

void DefineObstacleRectangle(Astar *A,double x1,double y1,double x2,double y2, double x3,double y3, double x4,double y4); // Attention, sommets tjrs dans le sens horlogique en commencant en haut à gauche
void DefineObstacleCircle(Astar *A,double xcenter,double ycenter,double radius);
void DefineObstacleLine(Astar *A,double x1,double y1,double x2,double y2,double xA,double yA, double xD, double yD,double epsilon);
void DefineObstacleStraightLine(Astar *A,double x1,double y1,double x2,double y2,double epsilon);
void DefineEurobotTable(Astar *A,double epsilon);

NAMESPACE_CLOSE();
#endif /* Obstacles_gr1_hpp */
