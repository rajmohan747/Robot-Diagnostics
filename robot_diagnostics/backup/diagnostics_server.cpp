
#include "diagnostics_server.h"




DiagnosticsServer::~DiagnosticsServer()
{

}

bool DiagnosticsServer::onManagedConfigure()
{

    return true;
}

bool DiagnosticsServer::onManagedUnconfigure()
{

    return true;
}

bool DiagnosticsServer::onManagedStart()
{

    return true;
}

bool DiagnosticsServer::onManagedStop()
{

    return true;
}

bool DiagnosticsServer::onManagedPause()
{
    return true;
}

bool DiagnosticsServer::onManagedResume()
{
    return true;
}
// bool DiagnosticsServer::onManagedCreate()
// {
//     return true;
// }

// bool DiagnosticsServer::onManagedTerminate()
// {
//     return true;
// }


/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "diagnostics_server");
/* Create an object to DiagnosticsActionServer */
    //DiagnosticsServer diagnostics_server(argc, argv, "diagnostics_server");
    ros::NodeHandle n;
    ros::Rate rate(1);
    //diagnostics_server.init().runAsync(0);

    while(ros::ok())
    {
      
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}