#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ebug_swarm_msg/dataArray.h"
#include "ebug_swarm_msg/frame.h"

#include "EBugData.h"
using namespace std;

#define PRINT_DEBUG 1

point points[MAX_BLOBS];

ros::Publisher *pubPtr;

void dataReceived(const ebug_swarm_msg::frame::ConstPtr& frameIn){
    ROS_INFO_STREAM("Localization node: blobs arrived and poses sent. Timestamp: " 
        << frameIn->timeStamp);
    vector<eBug> eBugs;
    int n_blobs = 0;
    int count = 0;
    // cout << "CHECKPOINT 1!  " << points[1].x << ":" << points[1].y << endl;
    // cout << "CHECKPOINT 10!  " << points[10].x << ":" << points[10].y << endl;
    // cout << "CHECKPOINT 250!  " << points[250].x << ":" << points[250].y << endl;
    for (int i = 0; i < frameIn->blobs.size(); ++i) {
      
      points[n_blobs].x      = (float)frameIn->blobs.at(i).x;
      points[n_blobs].y      = (float)frameIn->blobs.at(i).y;
      points[n_blobs].size   = (float)frameIn->blobs.at(i).radius;
      points[n_blobs].colour = frameIn->blobs.at(i).color;
      
      n_blobs++;
    }
    // cout << "RACE OVER 1!  " << points[1].x << ":" << points[1].y << endl;
    // cout << "RACE OVER 10!  " << points[10].x << ":" << points[10].y << endl;
    // cout << "RACE OVER 250!  " << points[250].x << ":" << points[250].y << endl;
    
    if (n_blobs >= 5) //apply K-nearest neighbour algorithm to classify groups of blobs as the eBugs
      {
	knn_graph_partition(n_blobs, eBugs, count);
      }

    // commentut << "size = " << eBugs.size() << endl;
    for (int i = 0; i < (int)eBugs.size(); i++)
      {
	cout << endl << "---------------eBugs Detected---------------------" << endl;
	// commentut << "ID =  " << eBugs[i].ID << "\t" << "x = " << eBugs[i].x_pos << "\t";
	cout << "y = " << eBugs[i].y_pos << "\t" << "angle = " << eBugs[i].angle << endl;
      }
    
    ebug_swarm_msg::dataArray arrayOut; // Should spit-out frames;
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
