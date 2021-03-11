/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#include <base_local_planner/local_planner_util.h>

#include <base_local_planner/goal_functions.h>

namespace base_local_planner 
{

void LocalPlannerUtil::initialize(
    tf2_ros::Buffer* tf,
    costmap_2d::Costmap2D* costmap,
    std::string global_frame) 
{
  if(!initialized_) 
  {
    tf_ = tf;
    costmap_ = costmap;
    global_frame_ = global_frame;
    initialized_ = true;
  }
  else
  {
    ROS_WARN("Planner utils have already been initialized, doing nothing.");
  }
}

void LocalPlannerUtil::reconfigureCB(LocalPlannerLimits &config, bool restore_defaults)
{
  if(setup_ && restore_defaults) {
    config = default_limits_;
  }

  if(!setup_) {
    default_limits_ = config;
    setup_ = true;
  }
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  limits_ = LocalPlannerLimits(config);
}

costmap_2d::Costmap2D* LocalPlannerUtil::getCostmap() {
  return costmap_;
}

LocalPlannerLimits LocalPlannerUtil::getCurrentLimits() {
  boost::mutex::scoped_lock l(limits_configuration_mutex_);
  return limits_;
}



bool LocalPlannerUtil::getGoal(geometry_msgs::PoseStamped& goal_pose) {
  //we assume the global goal is the last point in the global plan
  return base_local_planner::getGoalPose(*tf_,
        global_plan_,
        global_frame_,
        goal_pose);
}

bool LocalPlannerUtil::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) 
{
  if(!initialized_){
    ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
    return false;
  }

  //reset the global plan
  global_plan_.clear();

  global_plan_ = orig_global_plan;

  return true;
}

bool LocalPlannerUtil::getLocalPlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  //get the global plan in our frame
  if(!base_local_planner::transformGlobalPlan(
      *tf_,
      global_plan_,
      global_pose,
      *costmap_,
      global_frame_,
      transformed_plan)) {
    ROS_WARN("Could not transform the global plan to the frame of the controller");
    return false;
  }

  //now we'll prune the plan based on the position of the robot
  if(limits_.prune_plan) {
    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
  }
  return true;
}




} // namespace
