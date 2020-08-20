/**
* @file diagnostics_action_server.h
* @brief A diagnostic action server to execute actions like restart node, initimate process, alert warnings, etc.
*
* @author C Vipin Raj
* @version 1.0
* @date 20/04/20
*/

#ifndef MINIMAL_H_
#define MINIMAL_H_

// ROS Headers
#include "time.h"
#include "ros/ros.h"
#include "ros/console.h"

// System Headers
#include "string"
#include "stdlib.h"
#include "unistd.h"
#include "sys/reboot.h"
#include "dirent.h"
#include "sys/stat.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "boost/bind.hpp"
#include "thread"

// Other Libraries
#include "actionlib/server/simple_action_server.h"
#include "robot_activity/managed_robot_activity.h"
#include "robot_activity_msgs/State.h"
#include "monitoring_msgs/GuiInfo.h"
#include "std_srvs/Empty.h"


/**
 * @class DiagnosticsActionServer
 * @brief A Class implementation of action server for executing diagnostics actions with node lifecycle
 */

class Minimal : public robot_activity::ManagedRobotActivity
{
using ManagedRobotActivity::ManagedRobotActivity;
public:

    /**
     * @brief A node lifecycle, inheriting RobotActivity so the node APIs are automatically exposed
     * 
     */


    /**
     * @brief Default destructor, deletes all pointers and objects
     */
    ~Minimal();

    /**
     * @brief A managed node create function
     * 
     */
    void onManagedCreate() override;

    /**
     * @brief A managed node terminate function
     * 
     */
    void onManagedTerminate() override;

    /**
     * @brief A managed node configure function
     * 
     * @return true If action associated with node configure is executed 
     * @return false If node configure action is unsuccessful
     */
    bool onManagedConfigure() override;

    /**
     * @brief A managed node unconfigure function
     * 
     * @return true If action associated with node unconfigure is executed
     * @return false If node unconfigure action is unsuccessful
     */
    bool onManagedUnconfigure() override;

    /**
     * @brief A managed node start function
     * 
     * @return true If action associated with node start is executed 
     * @return false If node start action is unsuccessful
     */
    bool onManagedStart() override;

    /**
     * @brief A managed node stop function
     * 
     * @return true If action associated with node stop is executed
     * @return false If node stop action is unsuccessful
     */
    bool onManagedStop() override;

    /**
     * @brief A managed node resume function
     * 
     * @return true If action associated with node resume is executed
     * @return false If node resume action is unsuccessful
     */
    bool onManagedResume() override;

    /**
     * @brief A managed node pause function
     * 
     * @return true If action associated with node pause is executed
     * @return false If node pause action is unsuccessful
     */
    bool onManagedPause() override;

    
};

#endif /*DIAGNOSTICS_ACTION_SERVER_H_*/
