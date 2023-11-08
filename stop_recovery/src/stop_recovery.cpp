#include "stop_recovery.h"
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(stop_recovery::StopRecovery, nav_core::RecoveryBehavior)

namespace stop_recovery
{
StopRecovery::StopRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void StopRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    //ros::NodeHandle private_nh("~/" + name);
   // ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    
    //rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "rotational_vel", 0.0);
    //stop_velx_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_x", "stop_velx", 0.0);
    //top_velx_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_x", "stop_vely", 0.0);
    //blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

StopRecovery::~StopRecovery()
{
  delete world_model_;
}

void StopRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the StopRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Stop recovery behavior started.");

  ros::Rate r(10);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_angle = tf2::getYaw(global_pose.pose.orientation); //get current orientation and x y position commands.(usefull).
  double start_angle = current_angle;

    // Update Current Angle
  local_costmap_->getRobotPose(global_pose);
  current_angle = tf2::getYaw(global_pose.pose.orientation);

  double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

  ROS_INFO("cannot find a path. stopping the bot!");
  geometry_msgs::Twist cmd_vel;
 
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub.publish(cmd_vel);
  ROS_INFO("publishing");
  whle(true){
	ROS_INFO("IN RECOVERY MODE");
    }
  r.sleep();
  }
 
};  // namespace stop_recovery
