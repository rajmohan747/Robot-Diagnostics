#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "monitoring_core/monitor.h"
#include "monitoring_msgs/MonitoringArray.h"

using namespace sensor_msgs;
using namespace message_filters;

void callback(const monitoring_msgs::MonitoringArrayConstPtr& battery, const monitoring_msgs::MonitoringArrayConstPtr& imu)
{
  // Solve all of perception here...
  auto batteryErrors = battery->info[0].values;
  auto imuErrors = imu->info[0].values;
  ROS_INFO("Battery :%d  IMU : %d",batteryErrors.size(),imuErrors.size());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync");

  ros::NodeHandle nh;

  message_filters::Subscriber<monitoring_msgs::MonitoringArray> image_sub(nh, "battery", 1);
  message_filters::Subscriber<monitoring_msgs::MonitoringArray> info_sub(nh, "imu", 1);
  TimeSynchronizer<monitoring_msgs::MonitoringArray, monitoring_msgs::MonitoringArray> sync(image_sub, info_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
