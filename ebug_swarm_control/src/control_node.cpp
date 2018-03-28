#include "ebug_swarm_control/control_node.h"

using namespace std;

ros::Publisher *pubPtr;
std::string position;

void dataReceived(const ebug_swarm_msg::dataArray& arrayIn)
{
  ROS_INFO_STREAM("Control node: Commands received and blobs sent.");
} // dataReceived()

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;
  ros::Publisher blobs_pub = nh.advertise<std_msgs::String>("blobs", 1000);
  // Nicola's code:
  // pubPtr =
  //     new ros::Publisher(nh.advertise<ebug_swarm_msg::frame>("blobs",1000)); 
  // ros::Subscriber sub = nh.subscribe("commands", 1000, &dataReceived);
  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;

    ss << "blobs blobs blobs" << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    blobs_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    count++;
  } // while
  return 0;
} // main()
