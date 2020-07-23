#ifndef DIAGNOSTICS_SERVER_H
#define DIAGNOSTICS_SERVER_H

#include <iostream>
#include <ros/ros.h>
#include "robot_activity/managed_robot_activity.h"
#include "robot_activity_msgs/State.h"
#include "monitoring_msgs/GuiInfo.h"



class DiagnosticsServer: public robot_activity::ManagedRobotActivity
{
public:
    //using ManagedRobotActivity::ManagedRobotActivity;
    /* Constructor for the Ros class */ 
    //DiagnosticsServer();

     /* Destructor for the Ros class */
    ~DiagnosticsServer();
    bool onManagedConfigure() override;
    bool onManagedUnconfigure() override;
    bool onManagedStart() override;
    bool onManagedStop() override;
    bool onManagedPause() override;
    bool onManagedResume() override;
    //bool onManagedCreate() override;
    //bool onManagedTerminate() override;
};


#endif