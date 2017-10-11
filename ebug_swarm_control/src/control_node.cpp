#include "ebug_swarm_control/control_node.h"

ros::Publisher *pubPtr;

void dataReceived(const ebug_swarm_msg::dataArray& arrayIn){
    ROS_INFO_STREAM("Control node: Commands received and blobs sent. Timestamp: " 
        << arrayIn.timeStamp);
    ebug_swarm_msg::dataArray arrayOut;
    arrayOut = arrayIn;
    pubPtr->publish(arrayOut);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controlnode");
    
    ros::NodeHandle nh;
    
//     pubPtr = new ros::Publisher(
//         nh.advertise<sensor_msgs::Image>("imagecontrol",1000));    
    
    pubPtr = new ros::Publisher(
        nh.advertise<ebug_swarm_msg::dataArray>("blobs",1000));    
    
    
    
    ros::Subscriber sub = nh.subscribe("commands", 1000, &dataReceived);
    ros::spin();
    return 0;
}
