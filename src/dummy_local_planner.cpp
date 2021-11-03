#include <pluginlib/class_list_macros.h>

#include "dummy_local_planner.h"
#include <base_local_planner/goal_functions.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dummy_local_planner::DummyLocalPlanner, nav_core::BaseLocalPlanner)

namespace dummy_local_planner {

  void DummyLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      
      prune_plan_ = false;
      xy_goal_tolerance_ = 0.1;
      yaw_goal_tolerance_ = 6.28;
      rot_stopped_vel_ = 1e-2;
      trans_stopped_vel_ = 1e-2;

      tf_ = tf;

      costmap_ros_ = costmap_ros;

      ros::NodeHandle pn("~/" + name);


      //to get odometry information, we need to get a handle to the topic in the global namespace
      ros::NodeHandle gn;
      odom_sub_ = gn.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&DummyLocalPlanner::odomCallback, this, _1));

      initialized_ = true;
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }





  void DummyLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_mutex_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG_NAMED("dummy_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }





  bool DummyLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
/*
    tf::Stamped<tf::Pose> global_pose;
    if(!costmap_ros_->getRobotPose(global_pose))
      return false;

    costmap_2d::Costmap2D costmap;
    costmap_ros_->getCostmapCopy(costmap);
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if(!base_local_planner::transformGlobalPlan(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), transformed_plan)){
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);


    //we also want to clear the robot footprint from the costmap we're using
    costmap_ros_->clearRobotFootprint();


    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;


    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double yaw = tf::getYaw(goal_point.getRotation());

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if(base_local_planner::goalPositionReached(global_pose, goal_x, goal_y, xy_goal_tolerance_) || xy_tolerance_latch_){

      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;

      return true;
    }

    ROS_DEBUG_NAMED("dummy_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
    dp_->updatePlan(transformed_plan);

*/

    //pass along some dummy drive commands
    cmd_vel.linear.x = 0.2;
    cmd_vel.angular.z = 1.5;


    ROS_DEBUG_NAMED("dummy_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    return true;
  }





  bool DummyLocalPlanner::isGoalReached(){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
/*
    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_mutex_);
      base_odom = base_odom_;
    }

    return base_local_planner::isGoalReached(*tf_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID(), base_odom, 
        rot_stopped_vel_, trans_stopped_vel_, xy_goal_tolerance_, yaw_goal_tolerance_);
        */

    return false;
  }





  bool DummyLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;


    return true;
  }

};
