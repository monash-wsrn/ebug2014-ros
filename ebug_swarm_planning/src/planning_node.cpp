#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ebug_swarm_msg/arrayElement.h"
#include "ebug_swarm_msg/dataArray.h"


void dataReceived(const ebug_swarm_msg::dataArray& arrayIn){
    ROS_INFO_STREAM("Planning node: Poses came back to the origin: " << arrayIn.timeStamp);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning_node");
    
    ros::NodeHandle nh;
    
//     ros::Publisher pub = nh.advertise<sensor_msgs::Image>("imagestream", 1000);
    
    ros::Publisher pub = nh.advertise<ebug_swarm_msg::dataArray>("commands",1000);    
    
    int rate = 2;
    
    ros::Rate r(rate);
    
    int count = 0;
    ros::Subscriber sub = nh.subscribe("poses", 1000, &dataReceived);
    
    while (ros::ok())
    {
        
//         sensor_msgs::Image img;
        
        ebug_swarm_msg::arrayElement data;
        ebug_swarm_msg::dataArray array;
        
        array.timeStamp = 47;
        
        data.x = 320;
        data.y = 200;
        
        array.elements.push_back(data);
        
        data.x = 1024;
        data.y = 768;
        
         array.elements.push_back(data);
        
//         img.height = 480;
//         img.width = 640;
        
        /*
        msg.linear.x = double(rand())/double(RAND_MAX);
        msg.angular.z = 2*double(rand())/double(RAND_MAX)-1;
        */
        pub.publish(array);
        
//         ROS_INFO_STREAM("Sending Random velocity command:"
//             <<" linear==" << msg.linear.x
//             <<" angular==" <<msg.angular.z
//         );
//         
//         ros::spinOnce();
        ROS_INFO_STREAM("Data Sent. COUNTER: "<< count);
        
        
        ros::spinOnce();
        count++;
        r.sleep();
    }
    
    
//     ros::spin();
    return 0;
}
