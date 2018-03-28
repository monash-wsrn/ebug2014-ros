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

    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with
     * the type given as a template parameter to the advertise<>()
     * call, as was done in the constructor above.
     */
    blobs_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }

  return 0;
} // main()
