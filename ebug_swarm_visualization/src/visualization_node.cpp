#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ebug_swarm_msg/arrayElement.h"
#include "ebug_swarm_msg/dataArray.h"
#include <visualization_msgs/Marker.h>

ros::Publisher rviz_pub;

void dataReceived(const ebug_swarm_msg::dataArray::ConstPtr& arrayIn){
    ROS_INFO_STREAM("Visualization node: Poses Arrived at final destination: " << arrayIn->timeStamp);
    ROS_INFO_STREAM("Array Contents size: " << arrayIn->elements.size());
    for( int i=0; i<arrayIn->elements.size(); ++i )
    {
        const ebug_swarm_msg::arrayElement &data = arrayIn->elements[i];
        ROS_INFO_STREAM("\t" << data.x << " " << data.y );

	uint32_t shape = visualization_msgs::Marker::CYLINDER;
	visualization_msgs::Marker marker;
	marker.type = shape;
	marker.header.frame_id="/ebug_frame";
	marker.id = i;
	marker.ns = "ebug";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = data.x;
	marker.pose.position.y = data.y;
	marker.pose.position.z = 0;
	
	marker.scale.y = 0.6;
	marker.scale.x = 0.6;
	marker.scale.z = 0.2;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;

	// marker.lifetime = ros::Duration();
    ROS_INFO_STREAM("sending the ebug to rviz to see its position: " << marker.ns);

	rviz_pub.publish(marker);
	
        
    }

    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_node");
    
    ros::NodeHandle nh;
    rviz_pub = nh.advertise<visualization_msgs::Marker>("visualization_topic", 1);

    ros::Subscriber sub = nh.subscribe("poses", 1000, &dataReceived);

    
    ros::spin();
    return 0;
}
