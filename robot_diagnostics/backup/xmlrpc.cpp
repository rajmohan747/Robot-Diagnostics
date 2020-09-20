#include <iostream>
#include <ros/ros.h>
#include "xmlrpcpp/XmlRpcValue.h"
void readXmlrpc(ros::NodeHandle &nh)
{
    /* Node analyser parameter */
    XmlRpc::XmlRpcValue monitor_params;

    /* Read paramters for analysers */
    nh.getParam("/monitors", monitor_params);

    std::cout <<"Read  " << monitor_params << std::endl;

     //XmlRpc::XmlRpcValue::iterator xml_itr;

     for(auto xml_itr = monitor_params.begin();xml_itr!= monitor_params.end();xml_itr++)
     {
        std::cout <<"First  " << xml_itr->first << std::endl;
        XmlRpc::XmlRpcValue analyser_monitors = xml_itr->second["monitors"];
        XmlRpc::XmlRpcValue::iterator xml_itn;
            for (xml_itn = analyser_monitors.begin(); xml_itn != analyser_monitors.end(); ++xml_itn)
            {
                XmlRpc::XmlRpcValue monitors_type = xml_itn->first;
                std::cout << "Result  "<< monitors_type << std::endl;

            }
     }

}


/**
 * @brief Main function
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "xmlrpc");
    ros::NodeHandle n;
    ros::Rate rate(1);
  
    //boost::thread* controller_thread = new boost::thread(boost::bind(&NavigationWrapper::controlFlow, &controller));
   readXmlrpc(n);
    while(ros::ok())
    {
      
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}