
#include "trajectory.h"


Trajectory::Trajectory()
{
    /*Parameters from parameter server*/



    /*Publishers*/
    pathPub      = nh.advertise<nav_msgs::Path >("/path",1);
    //pathPub = nh.advertise<visualization_msgs::MarkerArray> ("visualization_marker_array", 0);
    
}  


Trajectory::~Trajectory()
{

}


void Trajectory::circleFormation(double radius,int shape)
{
    double x,y;
    std::vector<double> xVector;
    std::vector<double> yVector;



    double startingAngle,endingAngle;
    int direction;
    
    switch(shape)
    {
        case CircleShape::FULL:
            startingAngle = M_PI*-1;
            endingAngle   = M_PI;
            direction     = CC;
            break;

        case CircleShape::SEMITOP_CC:
            startingAngle = 0;
            endingAngle   = M_PI;
            direction     = CC;
            break; 
        case CircleShape::SEMIBOTTOM_CC:
            startingAngle = M_PI*-1;
            endingAngle   = 0;
            direction     = CC;
            break;                

    }
    for(double  theta = startingAngle;theta <= endingAngle ; theta = theta + (M_PI*0.1*direction))
    {
      // ROS_INFO("currrent theta    :%f",theta);
        x = radius*cos(theta);
        y = radius*sin(theta);

        xVector.push_back(x);
        yVector.push_back(y);

        //ROS_INFO("currrent theta    :%f   x :%f   y : %f",theta,x,y);

    }

    ROS_INFO("X :%d  Y :%d",xVector.size(),yVector.size());
    pathVisualization(xVector,yVector);



}


void Trajectory::pathVisualization(std::vector<double> &xVector,std::vector<double> &yVector)
{
    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "/map";
    
    for(int i =0; i < xVector.size(); i++)
    {

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id ="/map";

        pose.pose.position.x =xVector[i] ;
        pose.pose.position.y =yVector[i] ;


        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);

    }
    pathPub.publish(path);
}
/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Trajectory");
    ROS_INFO("Main of Trajectory called");
    
    Trajectory trajectory;

    ros::Rate rate(1);
    
    
    while(ros::ok())
    {        
        trajectory.circleFormation(1.0,2);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
