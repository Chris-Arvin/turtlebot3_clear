/*
 *      Author: tkruse
 * Note: 
    * 从这个.h可以看出，机器人是先执行到制定的position，然后原地旋转到指定orientation的。
    * 这里面的核心函数是：computeVelocityCommandsStopRotate()。
            在这里面：
              //如果position和orientation都到了，置0；
              //如果position到了，orientation没到，调用rotateToGoal；
              //如果position都没到，调用stopWithAccLimits。
    * obstacle_check的参数：第一个是目标位姿，第二个是当前的twist，第三个是目标的twist。（当前位姿它是已知的）
      系统模拟，看这个过程会不会撞障碍物（至于这种在机器人动力约束内能不能实现，那是我们自己需要提前考虑的）
    * 注意这里面引用赋值的cmd_vel是sim_period后的目标的twist（可以在动力上实现，且过程中不会碰障碍物），而不是我机器人现在要执行的
 */

#ifndef LATCHED_STOP_ROTATE_CONTROLLER_H_
#define LATCHED_STOP_ROTATE_CONTROLLER_H_

#include <string>

#include <Eigen/Core>

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>

namespace base_local_planner {

class LatchedStopRotateController {
public:
  LatchedStopRotateController(const std::string& name = "");
  virtual ~LatchedStopRotateController();
  //根据参数，判断是否到达了goal附近，这里只判断了position，没有考虑orientation
  bool isPositionReached(LocalPlannerUtil* planner_util,
                         const geometry_msgs::PoseStamped& global_pose);

  //这里不仅判断了position，还判断了orientation是否到达（这是判断，不管执行）
  bool isGoalReached(LocalPlannerUtil* planner_util,
      OdometryHelperRos& odom_helper,
      const geometry_msgs::PoseStamped& global_pose);

  void resetLatching() {
    xy_tolerance_latch_ = false;
  }


  /*
  * global_pose：当前位姿
  * robot_vel:stampedpose形式的当前twist
  * cmd_vel: 即将要被引用赋值的twist速度
  * sim_period: 仿真时间
  * obstacle_check：障碍物检测方法
  */
  //以某种加速度到达goal，同时考虑position和orientation
  //这个robot_vel是用posestamped来表示twist的一种方法，虽然不知道为啥这么搞。。
  //注意这个cmd_vel是sim_period后的目标的twist，而不是我机器人现在要执行的
  bool stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& robot_vel,
      geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

  //针对旋转。达到goal的position后旋转到指定orientation。
  //这个robot_vel是用posestamped来表示twist的一种方法，虽然不知道为啥这么搞。。
  //注意这个cmd_vel是sim_period后的目标的twist，而不是我机器人现在要执行的
  bool rotateToGoal(const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& robot_vel,
      double goal_th,
      geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      base_local_planner::LocalPlannerLimits& limits,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

  //对cmd_vel进行引用赋值
  //如果position和orientation都到了，置0；
  //如果position到了，orientation没到，调用rotateToGoal；
  //如果position都没到，调用stopWithAccLimits。
  //注意这个cmd_vel是sim_period后的目标的twist，而不是我机器人现在要执行的
  bool computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      LocalPlannerUtil* planner_util,
      OdometryHelperRos& odom_helper,
      const geometry_msgs::PoseStamped& global_pose,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

private:
  inline double sign(double x){
    return x < 0.0 ? -1.0 : 1.0;
  }


  // whether to latch at all, and whether in this turn we have already been in goal area
  bool latch_xy_goal_tolerance_;  //flag，是否允许xy坐标存在误差
  bool xy_tolerance_latch_;   //flag，是否position到达
  bool rotating_to_goal_;   //flag，是否orientation到达
};

} /* namespace base_local_planner */
#endif /* LATCHED_STOP_ROTATE_CONTROLLER_H_ */
