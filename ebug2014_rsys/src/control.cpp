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

    // This is a line from the logfile 1ebug_stationary_20sec_frames.txt
    // Normally the blob data  will come from the camera. 
    ss << "FRAME #" <<  count << ": [(612, 442, 1, 4), (633, 446, 0, 5), (588, 446, 2, 5), (655, 457, 1, 4), (570, 461, 2, 6), (669, 475, 2, 6), (556, 484, 0, 5), (676, 497, 2, 6), (555, 504, 2, 5), (673, 520, 2, 6), (562, 529, 1, 4), (663, 543, 1, 4), (575, 546, 2, 6), (644, 558, 1, 5), (620, 563, 1, 4), (596, 559, 1, 4)]";
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


