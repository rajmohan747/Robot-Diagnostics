#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <cstdio>
#include <ros/ros.h>



#include <stdio.h>
#include <vector>
#include <math.h>
#include <cmath>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/Path.h>

#define CW -1
#define CC 1

enum CircleShape
{
    FULL,
    SEMITOP_CC,
    SEMIBOTTOM_CC
};



class Trajectory
{
public:
    Trajectory();

     /* Destructor for the RosBagRecorder class */
    ~Trajectory();
    
    void circleFormation(double radius,int shape);

    void pathVisualization(std::vector<double> &xVector,std::vector<double> &yVector);
    


   
private:

    ros::NodeHandle nh;

    /*Subscribers*/

    /*Publishers*/
    
    ros::Publisher pathPub;



    /*Timer*/
    


     /*Member functions*/



    /*Member variables*/
   
};


#endif