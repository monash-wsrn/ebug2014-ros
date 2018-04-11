#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;
  ros::Publisher blobs_pub = nh.advertise<std_msgs::String>("blobs", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())  {
    std_msgs::String msg;
    std::stringstream ss;

    // This is a line from the logfile: 1ebug_85cm_moving_frames.log
    // Normally the blob data  will come from the camera. 
    ss << count << "2 1822945022  762  258  1  4    737  256  0  3    717  262  1  4    780  267  2  5    698  276  2  5    796  285  2  4    687  296  2  5    803  306  2  5    680  318  0  1    688  321  0  2    804  330  1  4    693  340  2  5    791  350  1  4    709  361  1  4    728  368  2  6    777  366  1  4    753  373  1  4";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    blobs_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }

  return 0;
} // main()


