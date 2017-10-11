#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ebug_swarm_msg/arrayElement.h"
#include "ebug_swarm_msg/dataArray.h"


void dataReceived(const ebug_swarm_msg::dataArray::ConstPtr& arrayIn){
    ROS_INFO_STREAM("Visualization node: Poses Arrived at final destination: " << arrayIn->timeStamp);
    ROS_INFO_STREAM("Array Contents");
    for( int i=0; i<arrayIn->elements.size(); ++i )
    {
        const ebug_swarm_msg::arrayElement &data = arrayIn->elements[i];
        ROS_INFO_STREAM("\t" << data.x << " " << data.y );
        
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_node");
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("poses", 1000, &dataReceived);

    
    ros::spin();
    return 0;
}
