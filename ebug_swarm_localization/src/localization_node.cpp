#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ebug_swarm_msg/dataArray.h"
#include "ebug_swarm_msg/frame.h"


ros::Publisher *pubPtr;

void dataReceived(const ebug_swarm_msg::frame::ConstPtr& frameIn)
{
  ROS_INFO_STREAM("Localization node: blobs arrived and poses sent.");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization");  
  
  ros::NodeHandle nh;
  //  pubPtr = new ros::Publisher(
  //	      nh.advertise<ebug_swarm_msg::dataArray>("poses",1000));    
  // ros::Subscriber sub = nh.subscribe("blobs", 1000, &dataReceived);
  ros::spin();
  return 0;
}
