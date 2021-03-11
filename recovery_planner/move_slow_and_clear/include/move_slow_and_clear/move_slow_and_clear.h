/*********************************************************************
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef MOVE_SLOW_AND_CLEAR_MOVE_SLOW_AND_CLEAR_H_
#define MOVE_SLOW_AND_CLEAR_MOVE_SLOW_AND_CLEAR_H_

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/Reconfigure.h>

namespace move_slow_and_clear 
{
  class MoveSlowAndClear : public nav_core::RecoveryBehavior
  {
    public:
      MoveSlowAndClear();
      ~MoveSlowAndClear();

      //初始化参数
      void initialize (std::string n, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* global_costmap,
          costmap_2d::Costmap2DROS* local_costmap);

      //先清空地图，再限速行走
      void runBehavior();

    private:
      //设置机器人限度
      void setRobotSpeed(double trans_speed, double rot_speed);
      
      //检测距离，如果机器人已经限速走过的距离超过了limited_distance，则去除速度限制
      void distanceCheck(const ros::TimerEvent& e);

      //机器人限速开始的位置和机器人当前位置的距离
      double getSqDistance();
      
      //去除机器人速度限制
      void removeSpeedLimit();

      ros::NodeHandle private_nh_, planner_nh_; //handle，第一个用于接收信息，第二个用于reconfigure
      costmap_2d::Costmap2DROS* global_costmap_;  //全局地图
      costmap_2d::Costmap2DROS* local_costmap_; //局部地图
      bool initialized_;  //是否被初始化
      double clearing_distance_, limited_distance_; //清空距离，限速行走的距离
      double limited_trans_speed_, limited_rot_speed_, old_trans_speed_, old_rot_speed_;  //限制的线速度，限制的角速度；旧的线速度、角速度
      ros::Timer distance_check_timer_;
      geometry_msgs::PoseStamped speed_limit_pose_; //机器人限速开始行走时的位姿
      boost::thread* remove_limit_thread_;  //移除限速用的线程
      boost::mutex mutex_;  //线程
      bool limit_set_;
      ros::ServiceClient planner_dynamic_reconfigure_service_;
  };
};

#endif
