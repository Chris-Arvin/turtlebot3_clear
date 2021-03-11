/*
 * latched_stop_rotate_controller.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: tkruse
 */

#include <base_local_planner/latched_stop_rotate_controller.h>

#include <cmath>

#include <Eigen/Core>

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_limits.h>

#include <tf2/utils.h>

namespace base_local_planner {

LatchedStopRotateController::LatchedStopRotateController(const std::string& name) 
{
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

  rotating_to_goal_ = false;
}

LatchedStopRotateController::~LatchedStopRotateController() {}


/**
 * returns true if we have passed the goal position.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 * Also goal orientation might not yet be true
 */
//
bool LatchedStopRotateController::isPositionReached(LocalPlannerUtil* planner_util,
    const geometry_msgs::PoseStamped& global_pose) 
{
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;

  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;

  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    xy_tolerance_latch_ = true;
    return true;
  }
  return false;
}


/**
 * returns true if we have passed the goal position and have reached goal orientation.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 */
bool LatchedStopRotateController::isGoalReached(LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper,
    const geometry_msgs::PoseStamped& global_pose) 
  {
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;
  double theta_stopped_vel = planner_util->getCurrentLimits().theta_stopped_vel;
  double trans_stopped_vel = planner_util->getCurrentLimits().trans_stopped_vel;

  //copy over the odometry information
  nav_msgs::Odometry base_odom;
  odom_helper.getOdom(base_odom);

  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;

  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_) {
      ROS_DEBUG("Goal position reached (check), stopping and turning in place");
      xy_tolerance_latch_ = true;
    }
    double goal_th = tf2::getYaw(goal_pose.pose.orientation);
    double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
    //check to see if the goal orientation has been reached
    if (fabs(angle) <= limits.yaw_goal_tolerance) {
      //make sure that we're actually stopped before returning success
      if (base_local_planner::stopped(base_odom, theta_stopped_vel, trans_stopped_vel)) {
        return true;
      }
    }
  }
  return false;
}

bool LatchedStopRotateController::stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose,
    const geometry_msgs::PoseStamped& robot_vel,
    geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) 
{

  //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
  //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
  double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim[0] * sim_period));
  double vy = sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim[1] * sim_period));

  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim[2] * sim_period));

  //we do want to check whether or not the command is valid
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
                                  Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
                                  Eigen::Vector3f(vx, vy, vth));

  //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
  if(valid_cmd)
  {
    ROS_DEBUG_NAMED("latched_stop_rotate", "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vth;
    return true;
  }
  ROS_WARN("Stopping cmd in collision");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return false;
}

bool LatchedStopRotateController::rotateToGoal(
    const geometry_msgs::PoseStamped& global_pose,
    const geometry_msgs::PoseStamped& robot_vel,
    double goal_th,
    geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    base_local_planner::LocalPlannerLimits& limits,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) 
{
  //目标角度
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  //当前角速度
  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  //角度差
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

  //机械结构的最大旋转角度限制（为啥用角度限制角速度啊。。）
  double v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, fabs(ang_diff)));

  //从当前角速度 在sim_period内能达到的最大和最小角速度
  double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * sim_period;
  double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * sim_period;
  //动力约束的最大角度限制（是否在sim_period能到）
  v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //最大角加速度进行匀a减速时需要的当前角速度
  double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff));
  v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));
  v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, v_theta_samp));

  if (ang_diff < 0) 
  {
    v_theta_samp = - v_theta_samp;
  }

  //这个obstacle_check的输入：第一个是目标位姿，第二个是当前的twist，第三个是目标的twist，看看这个过程会不会撞障碍物（至于这种在机器人动力约束内能不能实现，那是我们自己需要提前考虑的）
  //碰撞检测
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
      Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
      Eigen::Vector3f( 0.0, 0.0, v_theta_samp));

  if (valid_cmd) {
    ROS_DEBUG_NAMED("dwa_local_planner", "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
    cmd_vel.angular.z = v_theta_samp;
    return true;
  }
  ROS_WARN("Rotation cmd in collision");
  cmd_vel.angular.z = 0.0;
  return false;

}

//cmd_vel会被通过引用赋值
//global_pose是当前位姿
bool LatchedStopRotateController::computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper_,
    const geometry_msgs::PoseStamped& global_pose,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) 
{
  //获得goal的位姿
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) 
  {
    ROS_ERROR("Could not get goal pose");
    return false;
  }

  //动力约束
  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //到达了position，只需要进行旋转到达orientation就可以了
  if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_ ) 
  {
    ROS_INFO("Goal position reached, stopping and turning in place");
    xy_tolerance_latch_ = true;
  }

  //检查orientation
  double goal_th = tf2::getYaw(goal_pose.pose.orientation);
  double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
  //如果orientation也ok了，速度直接置0，其实就结束了
  if (fabs(angle) <= limits.yaw_goal_tolerance) 
  {
    //set the velocity command to zero
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    rotating_to_goal_ = false;
  } 

  //如果orientation不ok，现在开始进行旋转
  else 
  {
    ROS_DEBUG("Angle: %f Tolerance: %f", angle, limits.yaw_goal_tolerance);
    //获得当前速度
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    nav_msgs::Odometry base_odom;
    odom_helper_.getOdom(base_odom);

    //没停下，也没到goal
    if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, limits.theta_stopped_vel, limits.trans_stopped_vel)) 
    {
      /*
      * global_pose：当前位姿
      * robot_vel:stampedpose形式的当前twist
      * cmd_vel: 即将要被引用赋值的twist速度
      * sim_period: 仿真时间
      * obstacle_check：障碍物检测方法
      */
      //这里开始进行减速了
      if ( ! stopWithAccLimits(
          global_pose,
          robot_vel,
          cmd_vel,
          acc_lim,
          sim_period,
          obstacle_check)) 
      {
        ROS_INFO("Error when stopping.");
        return false;
      }
      ROS_DEBUG("Stopping...");
    }
    //如果已经停下了/到goal的position了，那么开始旋转
    else 
    {
      //旋转到目标角度
      rotating_to_goal_ = true;
      if ( ! rotateToGoal(
          global_pose,
          robot_vel,
          goal_th,
          cmd_vel,
          acc_lim,
          sim_period,
          limits,
          obstacle_check)) 
      {
        ROS_INFO("Error when rotating.");
        return false;
      }
      ROS_DEBUG("Rotating...");
    }
  }

  return true;

}


} /* namespace base_local_planner */
