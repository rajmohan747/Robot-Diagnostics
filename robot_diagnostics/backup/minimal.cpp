

#include "minimal.h"

Minimal::~Minimal()
{

}

void Minimal::onManagedCreate()
{
    /* TODO : Node action on create */
    ROS_INFO("onManagedCreate");
}

void Minimal::onManagedTerminate()
{
    /* Shutdown node on terminate */
    ROS_INFO("onManagedTerminate");
    if (ros::ok())
    {
        ros::shutdown();
    }
}

bool Minimal::onManagedConfigure()
{

    return true;
}

bool Minimal::onManagedUnconfigure()
{
    /* TODO : Node action on unconfigure */
    ROS_INFO("onManagedUnconfigure");
    return true;
}

bool Minimal::onManagedStart()
{
    /* Release all mutex lock and allow execution of diagnostics action */
    ROS_INFO("onManagedStart");

    return true;
}

bool Minimal::onManagedStop()
{
    /* Stop all action execution */
    ROS_INFO("onManagedStop");

    return true;
}

bool Minimal::onManagedPause()
{
    /* Stop all action execution */
    ROS_INFO("onManagedPause");

    return true;
}

bool Minimal::onManagedResume()
{
    /* Resume all action execution by releasing the locks */
    ROS_INFO("onManagedResume");

    return true;
}


int main(int argc, char **argv)
{
    /* Create an object to DiagnosticsActionServer */
    Minimal diagnosticNode(argc, argv, "minimal_node");

    /* Initialise the DiagnosticsActionServer node with node lifecycle */
    diagnosticNode.init().runAsync(0);

    /* Shutdown node initiated */
    ros::waitForShutdown();

    /* ROS sleep duration */
    ros::Duration(1.0).sleep();

    return 0;
}
