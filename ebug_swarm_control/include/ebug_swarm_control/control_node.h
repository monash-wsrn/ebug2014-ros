#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ebug_swarm_msg/arrayElement.h"
#include "ebug_swarm_msg/dataArray.h"
#include "ebug_swarm_msg/frame.h"
#include "ebug_swarm_msg/blob.h"
#include <iostream>
#include <fstream>
#include <string>

void dataReceived(const ebug_swarm_msg::dataArray& arrayIn);
