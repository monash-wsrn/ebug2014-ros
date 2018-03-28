#include "ros/ros.h"
#include "std_msgs/String.h"

void blobsCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
} // blobsCallback()

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("blobs", 1000, blobsCallback);
  ros::spin();
  return 0;
} // main()
