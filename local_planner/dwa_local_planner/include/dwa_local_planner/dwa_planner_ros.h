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
  *   在move_base中通过executeCb中的while循环来实现实时地动态规划和控制的。每次dwa被调用都会生成一个local best trajectory和当前的cmd_vel，然后pushlish cmd_vel给driver_base。执行完这一下就进入下一个循环，global plan然后control plan。。。如此反复
  *   最核心的函数是一个不断深入地调用： ComputeVelocityCommands --> dwaComputeVelocityCommands --> dp_->findBestPath --> gen_->nextTrajectory和scoreTrajectory --> generateTrajectory
  *   注意，我们的目标就是找到当前机器人应该执行的速度（cmd_vel），这个速度实际上是best traj在当前时间+dt时刻所对应的速度，而best traj的生成只是为了通过一步步的累积方针来找到当前状态下cmd_vel的（局部）最优解
  *   实际上，这里是存在误差的，机器人在每个dt后都会产生一个速度的跳变，但是因为dt很小，而且算法是实时的，所以可以忽略这个误差
*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_local_planner/dwa_planner.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlannerROS
   * @brief ROS Wrapper for the DWAPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class DWAPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      //初始化函数
      //setup_仅和reconfiguration相关，默认为false，表征是否re过;
      //initialized仅和isInitialized函数相关，表征是否被初始化过;
      DWAPlannerROS();

      //这个initialize核心是实例化了DWAPlanner了的dp_
      //真正的初始化是通过planner_util_来实现的
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~DWAPlannerROS();

      //引用赋值cmd_vel
      //实际上是调用了dwacomputeVelocityCommands
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      //引用赋值cmd_vel
      //最重要的是调用了dp_->findBestPath来求解local path和robot_vel（当前的tiwst）
      //注意cmd_vel通过pub与底盘交互
      //local plan的求解只是为了可视化
      bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);

      //这里是对base_local_planner的一个框架行的初始化，本质上就是调用dp_的setplan来把global的plan放进去
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      //判断是否达到了goal
      //核心是调用了latchedStopRotateController_，借助planner_util_,odom_helper_和curret_pose_来判断是否到了goal
      bool isGoalReached();



      bool isInitialized() {
        return initialized_;
      }

    private:
      //利用config进行重新配置
      //这个level没用到呀。。
      void reconfigureCB(DWAPlannerConfig &config, uint32_t level);

      //俩发布器，publishPlan是在goal_function中定义的
      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds

      //用于可视化的两个publisher，队列长度为1,用于可视化
      ros::Publisher g_plan_pub_, l_plan_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;
      //这个dp_是一个核心！！！！！
      boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller

      //地图
      costmap_2d::Costmap2DROS* costmap_ros_;
      //config的东西
      dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
      dwa_local_planner::DWAPlannerConfig default_config_;
      //是否reconfiguratino过
      bool setup_;
      //当期那pose，从costmap2DROS中获得
      geometry_msgs::PoseStamped current_pose_;

      //这个在isGoalReached中被使用
      base_local_planner::LatchedStopRotateController latchedStopRotateController_;

      //是否初始化完成
      bool initialized_;


      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
  };
};
#endif
