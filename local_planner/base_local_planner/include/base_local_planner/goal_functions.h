/*********************************************************************
* Author: Eitan Marder-Eppstein
* Developer: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Reference：https://www.cnblogs.com/sakabatou/p/8297479.html
* Note:
  *   在move_base中与dwa_planner交互的流程：
          1. 把global_planner规划出来的路径通过setplan赋给dwa；
          以下会进行循环：
          2. 调用isGoalReached，判断目标是否到达了目标点。如果没到，则调用computeVelocityCommands来规划局部速度(引用赋值cmd_vel)；
          3. 在computeVelocityCommands中，先将全局路径映射到局部地图中，然后调用updatePlanAndLocalCosts更新对应的打分项；
          4. 利用:isPostionReached判断是否达到了目标position：
              如果到了，computeVelocityCommandsStopRotate，来计算对应的orientation旋转；
              如果没到，调用dwaComputeVelocityCommands，引用赋值，计算dwa局部速度（一个cmd_vel）。
          5. 调用findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored = 0)，查找最优的局部路径
              先调用prepare对每个打分器进行准备；
              调用TrajectorySampleGenerator中的产生器列表（目前这个列表只有一个SimpleTrajectoryGenerator）在(vx,vy,w)离散空间内进行轨迹生成和打分：
                  在打分时，调用scoreTrajectory对打分项进行求和（如果某个打分器的scale参数是0，则说明该打分项无效，跳过这个；如果有某个打分项是负的，则这个轨迹行不通，直接return）。
  *   在局部规划时，会创建一个以机器人自己为中心的局部地图，并且将global_planner的goal裁减到局部地图的边界处，作为新的goal'
  *   打分对象共有6个：
          base_local_planner::OscillationCostFunction oscillation_costs_（摆动打分）
          base_local_planner::ObstacleCostFunction obstacle_costs_（避障打分）
          base_local_planner::MapGridCostFunction path_costs_（路径跟随打分）
          base_local_planner::MapGridCostFunction goal_costs_（指向目标打分）
          base_local_planner::MapGridCostFunction goal_front_costs_（前向点指向目标打分）
          base_local_planner::MapGridCostFunction alignment_costs_（对齐打分）
          1. oscillation_costs_：判断有三个，机器人只能前行与生成后退速度的摆动；低速时机器人角速度较大的摆动（尽量不让机器人原地转圈圈）；二轮机器人不能有侧移；
          2. obstacle_costs_：因为local costmap是不进行膨胀的，所以这里需要对每个点求解footprint，然后检测障碍物碰撞（本质上和膨胀再检测点是一样的，TODO或者这里可以优化，因为膨胀检测的速度更快）；
        3和4的stop_on_failure为true，即如果轨迹中的某个点为障碍物或者未在路径计算地图中探索到，则表示查找该点的得分失败，返回负值
          3. path_costs_：这里主要是考虑local best trajectory每个点到global_plan的距离（global_paln的每个点的dist都是0，然后广搜赋值其他点）；
          4. goal_costs_：是path_costs_的另类版，只考虑两个global_plan的goal和local best trajectory的goal之间的dist。（只让global_plan的goal的dist=0，然后广搜其他，本质上这里只需要考虑local goal的dist就好，但是为了格式统一，浪费了一些速度）；
        5和6的stop_on_failure为false，不考虑障碍物碰撞和坐标系转换了，原理分别和3、4一样，只是他们把当前点(x,y)进行了一个偏移（5是x_shift，6是y_shift），TODO本质上考虑的是某个点的舒适空间？。
          5. goal_front_costs_：同3，用(x_shift,y)
          6. alignment_costs_：同4，用(x,y_shift)
*********************************************************************/
#ifndef BASE_LOCAL_PLANNER_GOAL_FUNCTIONS_H_
#define BASE_LOCAL_PLANNER_GOAL_FUNCTIONS_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/buffer.h>

#include <string>
#include <cmath>

#include <angles/angles.h>
#include <costmap_2d/costmap_2d.h>

namespace base_local_planner {


  //计算global_pose到goal的距离，勾股定理。
  double getGoalPositionDistance(const geometry_msgs::PoseStamped& global_pose, double goal_x, double goal_y);


  //计算两个角的角度差，调用了angle::shortest_angular_distance
  double getGoalOrientationAngleDifference(const geometry_msgs::PoseStamped& global_pose, double goal_th);

  //给一个path和pub，用pub发布path
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);

  //删除已经执行过的plan
  void prunePlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan);

  //讲global_plan从全局的frame转换到局部costmap的frame，去除局部costmap外的点
  //注意，local planner的时候，只用local costmap
  bool transformGlobalPlan(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const geometry_msgs::PoseStamped& global_robot_pose,
      const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan);

  //global_plan是我们makeplan的路径点，求global_plan的最后一个点，引用赋值给goal_pose
  bool getGoalPose(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& global_frame,
      geometry_msgs::PoseStamped &goal_pose);

  //判断机器人是否到达了重点，先判断position和orientation，然后再判断odom，调用了基本上这个.h中所有的函数
  bool isGoalReached(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      geometry_msgs::PoseStamped& global_pose,
      const nav_msgs::Odometry& base_odom,
      double rot_stopped_vel, double trans_stopped_vel,
      double xy_goal_tolerance, double yaw_goal_tolerance);

  //读取odom的数据，和指定的值作对比，判断odom（机器人）是否真实停止了
  bool stopped(const nav_msgs::Odometry& base_odom, 
      const double& rot_stopped_velocity,
      const double& trans_stopped_velocity);
};
#endif
