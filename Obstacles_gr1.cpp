//
//  Obstacles_gr1.cpp
//
//
//  Created by William Chermanne on 13/03/17.
//
//
#include "ctrl_main_gr1.h"
#include "namespace_ctrl.h"
#include "Obstacles_gr1.hpp"
#include "CtrlStruct_gr1.h"
#include <math.h>
#include <stdlib.h>
#include "user_realtime.h"
#include "UsefulFunctions_gr1.hpp"
#include "Astar_gr1.hpp"

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define BLU   "\x1B[34m"
#define RESET "\x1B[0m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"

NAMESPACE_INIT(ctrlGr1);

/* This function defines a rectangle obstacle in the map of nodes
*
* param[in] : Astar, coordinates of the 4 corners of the rectangle
*/
void DefineObstacleRectangle(Astar *A,double x1,double y1,double x2,double y2, double x3,double y3, double x4,double y4) // Attention, sommets tjrs dans le sens horlogique en commencant en haut à gauche
{
  printf("DEFINING OBSTACLE RECTANGLE...\n");

  double xP,yP;
  double P1,P2,P3,P4;
  double p1,p2,p3,p4;

  double a1,a2,a3,a4;
  double b1,b2,b3,b4;
  double c1,c2,c3,c4;
  double d1,d2,d3,d4;

  double SurfRect;
  double S1,S2,S3,S4;

  a1=EuclidianDistance(x1,y1,x2,y2);
  a2=EuclidianDistance(x2,y2,x3,y3);
  a3=EuclidianDistance(x3,y3,x4,y4);
  a4=EuclidianDistance(x4,y4,x1,y1);

  SurfRect=a1*a2;

  for (int i=0;i<A->nbr_nodes;i++) // Parcourir map et checker si le point est bien à l'intérieur du rectangle
  {
    printf("Considering Node %d !\n",i);

    // point p
    xP = A->map[i]->x;
    yP = A->map[i]->y;

    // Triangle a,p,b
    b1=EuclidianDistance(x1,y1,xP,yP);
    c1=EuclidianDistance(x2,y2,xP,yP);
    P1=a1+b1+c1;
    p1=P1/2;
    S1=double(sqrt(p1*(p1-a1)*(p1-b1)*(p1-c1))); // On a S1 MDR

    // Triangle b,p,c
    b2=EuclidianDistance(x2,y2,xP,yP);
    c2=EuclidianDistance(x3,y3,xP,yP);
    P2=a2+b2+c2;
    p2=P2/2;
    S2=double(sqrt(p2*(p2-a2)*(p2-b2)*(p2-c2))); // On a S2 MDR

    // Triangle c,p,d
    b3=EuclidianDistance(x3,y3,xP,yP);
    c3=EuclidianDistance(x4,y4,xP,yP);
    P3=a3+b3+c3;
    p3=P3/2;
    S3=double(sqrt(p3*(p3-a3)*(p3-b3)*(p3-c3))); // On a S3 MDR

    // Triangle c,p,d
    b4=EuclidianDistance(x4,y4,xP,yP);
    c4=EuclidianDistance(x1,y1,xP,yP);
    P4=a4+b4+c4;
    p4=P4/2;
    S4=double(sqrt(p4*(p4-a4)*(p4-b4)*(p4-c4))); // On a S4 MDR

    if(fabs((S1+S2+S3+S4)-SurfRect)<0.01)
    {
      A->map[i]->walkable=0;
    }
    else
    {

    }
  }
}

/* This function defines a circle obstacle in the map of nodes
*
* param[in] : Astar, coordinates of the center and radius of the circle
*/
void DefineObstacleCircle(Astar *A,double xcenter,double ycenter,double radius)
{

  double distance_center_point;
  double xP,yP;
  for (int i=0;i<A->nbr_nodes;i++)
  {
    xP = A->map[i]->x;
    yP = A->map[i]->y;
    distance_center_point=EuclidianDistance(xP,yP,xcenter,ycenter);
    if(distance_center_point-radius<0.01)
    {
      A->map[i]->walkable=0;
    }

  }

}



void DefineObstacleLine(Astar *A,double x1,double y1,double x2,double y2,double xA,double yA, double xD, double yD,double epsilon)
{
  double thickness=68;
  double a=(y2-y1)/(x2-x1);
  double b= y1-a*x1;
  double xP,yP;
  //double xmin=fmin(x1,x2);


  double a1=(yD-yA)/(xD-xA);
  double b1= y1-a1*x1;


  for (int i=0;i<A->nbr_nodes;i++)
  {
    xP = A->map[i]->x;
    yP = A->map[i]->y;
    if((xP>(yP-b1)/a1) && fabs(yP-(a*xP+b))<(thickness+epsilon))
    {
      A->map[i]->walkable=0;
    }

  }
}

/* This function returns the index of the node having the lowest cost in the open list
*
* param[in] : Astar and target node
* param[out] : Array with indexes of the nodes in the path
*/
void DefineObstacleStraightLine(Astar *A,double x1,double y1,double x2,double y2,double epsilon)
{
  double thickness=68;
  double a=(y2-y1)/(x2-x1);
  double b= y1-a*x1;
  double xP,yP;
  double xmin=fmin(x1,x2);

  for (int i=0;i<A->nbr_nodes;i++)
  {
    xP = A->map[i]->x;
    yP = A->map[i]->y;
    if((xP>xmin && fabs(yP-(a*xP+b))<(thickness+epsilon)))
    {
      A->map[i]->walkable=0;
    }

  }
}

void DefineEurobotTable(Astar *A,double epsilon)
{


  // Wall Ouest
  double epaisseur_wall=11;
  double x1w1= -1022.0;
  double y1w1= 1522.0;

  double x2w1= -1000.0+epsilon;
  double y2w1= 1522.0;

  double x3w1= -1000.0+epsilon;
  double y3w1= -1522.0;

  double x4w1=-1022.0;
  double y4w1= -1522.0;

  DefineObstacleRectangle(A,x1w1,y1w1,x2w1,y2w1,x3w1,y3w1,x4w1,y4w1);


  // Wall Nord
  double x1w2= -1022.0;
  double y1w2= 1522.0;

  double x2w2= 1022.0;
  double y2w2= 1522.0;

  double x3w2= 1022.0;
  double y3w2= 1500.0-epsilon;

  double x4w2=-1022.0;
  double y4w2= 1500.0-epsilon;

  DefineObstacleRectangle(A,x1w2,y1w2,x2w2,y2w2,x3w2,y3w2,x4w2,y4w2);


  // Wall Est
  double x1w3= 1000.0-epsilon;
  double y1w3= 1522.0;

  double x2w3= 1022.0;
  double y2w3= 1522.0;

  double x3w3= 1022.0;
  double y3w3= -1522.0;

  double x4w3=1000-epsilon;
  double y4w3= -1522.0;

  DefineObstacleRectangle(A,x1w3,y1w3,x2w3,y2w3,x3w3,y3w3,x4w3,y4w3);


  // Wall Sud
  double x1w4= -1022.0;
  double y1w4= -1500.0+epsilon;

  double x2w4= 1022.0;
  double y2w4= -1500.0+epsilon;

  double x3w4= 1022.0;
  double y3w4= -1522.0;

  double x4w4=-1022.0;
  double y4w4= -1522.0;

  DefineObstacleRectangle(A,x1w4,y1w4,x2w4,y2w4,x3w4,y3w4,x4w4,y4w4);



  // Starting area wall 1
  double x1saw1= -760.0-epsilon;
  double y1saw1= -830.0+epsilon;

  double x2saw1= -498.0+epsilon;
  double y2saw1= -830.0+epsilon;

  double x3saw1= -498.0+epsilon;
  double y3saw1= -1522.0;

  double x4saw1= -760.0-epsilon;
  double y4saw1= -1522.0;

  DefineObstacleRectangle(A,x1saw1,y1saw1,x2saw1,y2saw1,x3saw1,y3saw1,x4saw1,y4saw1);


  // Starting area wall 2
  double x1saw2= -640.0-epsilon;
  double y1saw2= 1522;

  double x2saw2= -618.0+epsilon;
  double y2saw2= 1522.0;

  double x3saw2= -618.0+epsilon;
  double y3saw2= 500.0-epsilon;

  double x4saw2= -640.0-epsilon;
  double y4saw2= 500.0-epsilon;

  DefineObstacleRectangle(A,x1saw2,y1saw2,x2saw2,y2saw2,x3saw2,y3saw2,x4saw2,y4saw2);

  // Big circle at the middle right

  double rfusee=180+epsilon;

  double rcircle=200+epsilon;
  double xcircle=1000;
  double ycircle=0;
  DefineObstacleCircle(A,xcircle,ycircle,rcircle);


  // Rectangles du Big circle 1
  double x1RBC1= 200;
  double y1RBC1= 68;

  double x2RBC1= 1000;
  double y2RBC1= 68;

  double x3RBC1= 1000;
  double y3RBC1= -68;

  double x4RBC1= 200;
  double y4RBC1= -68;
  DefineObstacleRectangle(A,x1RBC1,y1RBC1,x2RBC1,y2RBC1,x3RBC1,y3RBC1,x4RBC1,y4RBC1);

  // Rectangles du Big circle 2
  double x1RBC2= 482.4;
  double y1RBC2= 613.8;

  double x2RBC2= 1000;
  double y2RBC2= 68;

  double x3RBC2= 1000;
  double y3RBC2= -68;

  double x4RBC2= 386.2;
  double y4RBC2= 517.6;
  DefineObstacleRectangle(A,x1RBC2,y1RBC2,x2RBC2,y2RBC2,x3RBC2,y3RBC2,x4RBC2,y4RBC2);

  // Rectangles du Big circle 3
  double x1RBC3= 482.4;
  double y1RBC3= -613.8;

  double x2RBC3= 1000;
  double y2RBC3= 68;

  double x3RBC3= 1000;
  double y3RBC3= -68;

  double x4RBC3= 386.2;
  double y4RBC3= -517.6;
  DefineObstacleRectangle(A,x1RBC3,y1RBC3,x2RBC3,y2RBC3,x3RBC3,y3RBC3,x4RBC3,y4RBC3);

  // Fusée 1

  double rf1=rfusee;
  double xf1=-960;
  double yf1=-350;

  DefineObstacleCircle(A,xf1,yf1,rf1);

  // Fusée 2

  double rf2=rfusee;
  double xf2=-960;
  double yf2=350;

  DefineObstacleCircle(A,xf2,yf2,rf2);

  // Fusée 3

  double rf3=rfusee;
  double xf3=350;
  double yf3=1460;

  DefineObstacleCircle(A,xf3,yf3,rf3);

  // Fusée 4

  double rf4=rfusee;
  double xf4=350;
  double yf4=-1460;

  DefineObstacleCircle(A,xf4,yf4,rf4);

  // Rectangle Module 1
  double x1RM1= -322;
  double y1RM1= 1522;

  double x2RM1= 172;
  double y2RM1= 1522;

  double x3RM1= 172;
  double y3RM1= 1414;

  double x4RM1= -322;
  double y4RM1= 1414;
  DefineObstacleRectangle(A,x1RM1,y1RM1,x2RM1,y2RM1,x3RM1,y3RM1,x4RM1,y4RM1);


  // Rectangle Module 2
  double x1RM2= -322;
  double y1RM2= -1414;

  double x2RM2= 172;
  double y2RM2= -1414;

  double x3RM2= 172;
  double y3RM2= -1522;

  double x4RM2= -322;
  double y4RM2= -1522;
  DefineObstacleRectangle(A,x1RM2,y1RM2,x2RM2,y2RM2,x3RM2,y3RM2,x4RM2,y4RM2);


  double xA1=442.4;
  double yA1=613.4;
  double xD1=386.2;
  double yD1=517.6;

  double xA2=442.4;
  double yA2=-613.4;
  double xD2=386.2;
  double yD2=-517.6;
  DefineObstacleStraightLine(A,200.0,0.0,1000.0,0.0,0.0);
  DefineObstacleLine(A,434.3,565.7,1000.0,0.0,xA1,yA1,xD1,yD1,20.0);
  DefineObstacleLine(A,434.3,-565.7,1000.0,0.0,xA2,yA2,xD2,yD2,20.0);


}
NAMESPACE_CLOSE();
