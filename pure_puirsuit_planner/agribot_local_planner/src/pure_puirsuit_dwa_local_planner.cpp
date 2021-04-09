/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "agribot_local_planner.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TransformStamped.h>
#include <base_local_planner/goal_functions.h>

using namespace std;

// register planner
PLUGINLIB_EXPORT_CLASS(agribot_local_planner::AgribotLocalPlanner,nav_core::BaseLocalPlanner)

namespace agribot_local_planner {

constexpr double kControllerFrequency = 20.0;

AgribotLocalPlanner::AgribotLocalPlanner()
    : initialized_(false), goal_reached_(false),tf_listener_(tf_buffer_),ld_(1.0), v_max_(0.1), v_(v_max_), w_max_(1.0), pos_tol_(0.3),
      ori_tol_(3.14), map_frame_id_("map"), robot_frame_id_("base_link"),
       lookahead_frame_id_("lookahead") {}

AgribotLocalPlanner::~AgribotLocalPlanner() {
}

template<typename T1, typename T2>
double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }

void AgribotLocalPlanner::initialize(std::string name,tf::TransformListener* tf,costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    tf_ = tf;
    odom_helper_ = new base_local_planner::OdometryHelperRos(
        "/sensor_pose_odo");  //  /sensor_pose_odo
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/target_pose", 10);  // target_pose
    curr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/current_pose", 10);  // target_pose

    base_frame_ = "base_footprint";
    plan_index_ = 0;
    last_plan_index_ = 0;
    goal_reached_ = false;
    initialized_ = true;

    costmap_ros_ = costmap_ros;
    // initialize the copy of the costmap the controller will use
    costmap_ = costmap_ros_->getCostmap();

    // time interval
    double controller_freqency;
     // Vehicle parameters
  
  nh.param("/move_base/controller_frequency", controller_freqency,
             kControllerFrequency);

  nh.param<double>("lookahead_distance", ld_, 0.5);
  nh.param<double>("max_rotational_velocity", w_max_, 1.2);
  nh.param<double>("position_tolerance", pos_tol_, 0.05);
  nh.param<string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  nh.param<string>("robot_frame_id", robot_frame_id_, "base_footprint");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");
  // Frame attached to midpoint of front axle (for front-steered vehicles).
  nh.param<string>("model_frame_id", model_frame_id_, "rear_axle_midpoint");

  d_t_ = 1 / controller_freqency;

  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;
  
    ROS_INFO("Agribot local planner initialized!");
  } else {
    ROS_WARN("Agribot local planner has already been initialized.");
  }
}

// plugin functions
bool AgribotLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
  if (!initialized_) {
    ROS_ERROR("Agribot local planner has not been initialized.");
    return false;
  }
  // set new plan
  global_plan_.clear();
  global_plan_ = global_plan;
  // reset plan parameters
  plan_index_ = 0;
  goal_reached_ = false;
  return true;
}

void AgribotLocalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path,const ros::Publisher& pub) {
  // given an empty path we won't do anything
  if (path.empty()) return;

  // create a path message
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = path[0].header.frame_id;
  gui_path.header.stamp = path[0].header.stamp;

  // Extract the plan in world co-ordinates, we assume the path is all in the
  // same frame
  for (unsigned int i = 0; i < path.size(); i++) {
    gui_path.poses[i] = path[i];
  }

  pub.publish(gui_path);
}

bool AgribotLocalPlanner::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR("Agribot local planner has not been initialized.");
    return false;
  }
  if (goal_reached_) {
    ROS_ERROR("AgribotLocalPlanner goal reached...");
    return true;
  }
  return goal_reached_;
}

KDL::Frame AgribotLocalPlanner::transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

  return F_map_tf.Inverse()*F_map_pose;
}

bool AgribotLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("pure puirsuit planner has not been initialized.");
    return false;
  }
  if (goal_reached_) {
    ROS_ERROR("AgribotLocalPlanner goal reached without motion.");
    return true;
  }
  // next target
  geometry_msgs::PoseStamped target;
  geometry_msgs::PoseStamped curr_pose;

  double t_x, t_y, t_th;
  double x_vel = 0, th_vel = 0;
  double t_th_w = 0.0;

  // looking for the next point in the path far enough with minimum differene in
  // angle
  geometry_msgs::TransformStamped tf;
  try
  { 
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
   
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; plan_index_ < global_plan_.size(); plan_index_++) {
    
     if (distance(global_plan_[plan_index_].pose.position, tf.transform.translation) > ld_)
      {

        // Transformed lookahead to base_link frame is lateral error
        KDL::Frame F_bl_ld = transformToBaseLink(global_plan_[plan_index_].pose, tf.transform);
        lookahead_.transform.translation.x = F_bl_ld.p.x();
        lookahead_.transform.translation.y = F_bl_ld.p.y();
        lookahead_.transform.translation.z = F_bl_ld.p.z();
        F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);
        
        // TODO: See how the above conversion can be done more elegantly
        // using tf2_kdl and tf2_geometry_msgs

        break;
      }
    }
    

   if (!global_plan_.empty() && plan_index_ >= global_plan_.size())
    {
      // We are approaching the goal,
      // which is closer than ld

      // This is the pose of the goal w.r.t. the base_link frame
      KDL::Frame F_bl_end = transformToBaseLink(global_plan_.back().pose, tf.transform);
      double roll, pitch, yaw;
      F_bl_end.M.GetRPY(roll, pitch, yaw);
      ROS_INFO("yaw %f",yaw);
      if ((fabs(F_bl_end.p.x()) <= pos_tol_) && (yaw <= ori_tol_)) 
      {
        // We have reached the goal
        goal_reached_ = true;
        //std_msgs::Bool goal_r ;
        //goal_r.data = true ;
        ROS_WARN_STREAM("goal reached published !");
        //pub_state_.publish(goal_r);
        
        // Reset the path
        //global_plan_ = nav_msgs::Path();
      }
      else
      {
        // We need to extend the lookahead distance
        // beyond the goal point.
      
        // Find the intersection between the circle of radius ld
        // centered at the robot (origin)
        // and the line defined by the last path pose
        /*double k_end = tan(yaw); // Slope of line defined by the last path pose
        double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
        double a = 1 + k_end * k_end;
        double b = 2 * l_end;
        double c = l_end * l_end - ld_ * ld_;
        double D = sqrt(b*b - 4*a*c);
        double x_ld = (-b + copysign(D,v_)) / (2*a);
        double y_ld = k_end * x_ld + l_end;
        ROS_INFO("x_ld %f y_ld %f",x_ld,y_ld);
        
        lookahead_.transform.translation.x = x_ld;
        lookahead_.transform.translation.y = y_ld;
        lookahead_.transform.translation.z = F_bl_end.p.z();
        F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x,
                                 lookahead_.transform.rotation.y,
                                 lookahead_.transform.rotation.z,
                                 lookahead_.transform.rotation.w);*/
      }
    }

    if (!goal_reached_)
    {
      // We are tracking.

      // Compute linear velocity.
      // Right now,this is not very smart :)
      v_ = copysign(v_max_, v_); 
      //ROS_INFO("linerar %f",v_);
      
      // Compute the angular velocity.
      // Lateral error is the y-value of the lookahead point (in base_link frame)
      double yt = lookahead_.transform.translation.y; 
      double xt = lookahead_.transform.translation.x;//ROS_INFO("yt %f",yt);
      double ld_2 = ld_ * ld_;
      double exp = 2*v_ / ld_2 * yt;
      double ang_ = copysign(std::min( fabs(exp) , w_max_ ),exp);
      cmd_vel.angular.z = ang_ ;

      // Compute desired Ackermann steering angle
      //cmd_acker_.drive.steering_angle = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );
      
      // Set linear velocity for tracking.
      double linv_ = std::min( fabs(xt / yt)* 0.1 , 0.1) ;
      //if(fabs(ang_) == w_max_ ){linv_ = 0.0;}
      cmd_vel.linear.x = linv_;
      ROS_INFO("angular %f yt %f xt %f v_ %f",ang_,yt,xt,linv_);

    }
    else
    {
      // We are at the goal!

      // Stop the vehicle
      
      // The lookahead target is at our current pose.
      lookahead_.transform = geometry_msgs::Transform();
      lookahead_.transform.rotation.w = 1.0;
      
      // Stop moving.
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;

    }

    // Publish the lookahead target transform.
    lookahead_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(lookahead_);
    
  
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
  }

  return true;
}
};  // namespace agribot_local_planner