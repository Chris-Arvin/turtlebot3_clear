/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#ifndef ABSTRACT_LOCAL_PLANNER_ODOM_H_
#define ABSTRACT_LOCAL_PLANNER_ODOM_H_

#include <nav_core/base_local_planner.h>

#include <boost/thread.hpp>

#include <costmap_2d/costmap_2d.h>
#include <tf2_ros/buffer.h>

#include <base_local_planner/local_planner_limits.h>


namespace base_local_planner {

/**
 * @class LocalPlannerUtil
 * @brief Helper class implementing infrastructure code many local planner implementations may need.
 */
class LocalPlannerUtil {

private:
  // things we get from move_base
  std::string name_;
  std::string global_frame_;

  costmap_2d::Costmap2D* costmap_;
  tf2_ros::Buffer* tf_;


  std::vector<geometry_msgs::PoseStamped> global_plan_;


  boost::mutex limits_configuration_mutex_;
  bool setup_;
  LocalPlannerLimits default_limits_;
  LocalPlannerLimits limits_;
  bool initialized_;

public:

  LocalPlannerUtil() : initialized_(false) {}

  ~LocalPlannerUtil() {
  }

  void reconfigureCB(LocalPlannerLimits &config, bool restore_defaults);
  
  //非重复性地做初始化  
  void initialize(tf2_ros::Buffer* tf,
      costmap_2d::Costmap2D* costmap,
      std::string global_frame);
  
  //调用getgoalpose，引用赋值goal_pose
  bool getGoal(geometry_msgs::PoseStamped& goal_pose);

  //重新设定global_plan_
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  //引用赋值，将全局的plan转换到局部（机器人当前位姿为起点）
  bool getLocalPlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan);

  costmap_2d::Costmap2D* getCostmap();

  LocalPlannerLimits getCurrentLimits();

  std::string getGlobalFrame(){ return global_frame_; }
};




};

#endif /* ABSTRACT_LOCAL_PLANNER_ODOM_H_ */
