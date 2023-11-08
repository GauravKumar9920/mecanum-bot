#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float lx = 0.25925;
float ly = 0.225;
int ticks_per_rotation = 10000;
float gear_ratio = 20;
float wheel_radius = 0.1015;
float ticks_meter = (ticks_per_rotation * gear_ratio) / (2 * M_PI);

int encoder_max = 2147483647;
int encoder_min = -2147483648;
float encoder_high_wrap = (encoder_max - encoder_min) * 0.7 + encoder_min;
float encoder_low_wrap = (encoder_max - encoder_min) * 0.3 + encoder_min;

int front_left_enc, front_right_enc, back_left_enc, back_right_enc;
int last_front_left_enc, last_front_right_enc, last_back_left_enc, last_back_right_enc;

int frult = 0;
int flult = 0;
int blult = 0;
int brult = 0;

float w1, w2, w3, w4, vx, vy, vth;
float dx, dy;
float x = 0.0;
float y = 0.0;
float th = 0.0;
static float init_th;
ros::Publisher odom_pub;

ros::Time current_time, last_time;
bool init = 1;
bool new_fl = 0;
bool new_fr = 0;
bool new_bl = 0;
bool new_br = 0;
bool got_imu = 0;

