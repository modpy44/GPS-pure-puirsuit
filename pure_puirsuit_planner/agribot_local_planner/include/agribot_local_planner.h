/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_
#define ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_core/base_local_planner.h>
#include <kdl/frames.hpp>

#include <fstream>
#include <string>
#include <vector>

#include "agribot_local_planner/AgribotLocalPlannerConfig.h"

namespace agribot_local_planner {

class AgribotLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  AgribotLocalPlanner();
  ~AgribotLocalPlanner();

  // base local planner plugin functions
  void initialize(std::string name, tf::TransformListener *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan);

  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

  bool isGoalReached();

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
  KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf);


 private:
  // callbacks
  void emergencyStopCallback(const std_msgs::Bool::ConstPtr &stop_msg);

  // actions
  void robotStops() {
    goal_reached_ = true;
    ROS_INFO("Robot will stop.");
  }

  costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
  costmap_2d::Costmap2D* costmap_; ///< @brief The costmap the controller will use

   tf::Vector3 robot_curr_pose;
   double robot_curr_orien;
  std::vector<double> final_orientation;

  tf::TransformListener *tf_;
  base_local_planner::OdometryHelperRos *odom_helper_;
  // publishers
  ros::Publisher target_pose_pub_, curr_pose_pub;
  // subscribers
  ros::Subscriber emergency_stop_sub_;
  //ros::Subscriber odom_sub;

  // input params
  std::string base_frame_;
  tf2_ros::TransformListener tf_listener_ ;
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tol_ , ori_tol_;
  // Generic control variables
  double v_max_, v_, w_max_;
  // Control variables for Ackermann steering
  // Steering angle is denoted by delta
  double delta_, delta_vel_, acc_, jerk_, delta_max_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  std::string map_frame_id_, robot_frame_id_, lookahead_frame_id_ ,model_frame_id_ ;
  bool initialized_, goal_reached_,rotating_to_goal_;
  int plan_index_, last_plan_index_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  double d_t_;

};
}  // namespace agribot_local_planner

#endif  // ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_
