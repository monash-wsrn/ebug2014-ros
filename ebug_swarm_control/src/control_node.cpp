#include "ebug_swarm_control/control_node.h"

using namespace std;

ros::Publisher *pubPtr;
std::string position;

void dataReceived(const ebug_swarm_msg::dataArray& arrayIn){

  ROS_INFO_STREAM("Control node: Commands received and blobs sent.");
  
} // dataReceived()

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controlnode");
  
  ros::NodeHandle nh;
    
  
  pubPtr = new ros::Publisher(nh.advertise<ebug_swarm_msg::frame>("blobs",1000));    
  
    
  ros::Subscriber sub = nh.subscribe("commands", 1000, &dataReceived);
  ros::spin();
  return 0;
}
