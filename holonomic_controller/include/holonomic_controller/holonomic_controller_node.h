#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <map>
#include <jsoncpp/json/json.h>

ros::Publisher speed_pub;
std_msgs::String msg;

float lx = 0.25925;
float ly = 0.225;
float wheel_dia = 0.203;
float wheel_radius = wheel_dia / 2;

float w1;
float w2;
float w3;
float w4;