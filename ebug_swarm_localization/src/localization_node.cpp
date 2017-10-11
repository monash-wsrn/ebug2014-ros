#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ebug_swarm_msg/dataArray.h"

ros::Publisher *pubPtr;

void dataReceived(const ebug_swarm_msg::dataArray::ConstPtr& arrayIn){
    ROS_INFO_STREAM("Localization node: blobs arrived and poses sent. Timestamp: " 
        << arrayIn->timeStamp);
    ebug_swarm_msg::dataArray arrayOut;
    arrayOut = *arrayIn;
    pubPtr->publish(arrayOut);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_node");
    
    ros::NodeHandle nh;
    pubPtr = new ros::Publisher(
        nh.advertise<ebug_swarm_msg::dataArray>("poses",1000));    
    ros::Subscriber sub = nh.subscribe("blobs", 1000, &dataReceived);    
    ros::spin();
    return 0;
}
