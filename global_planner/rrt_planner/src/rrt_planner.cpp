/*********************************************************************
* Authors: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update date:2020.7.15
*********************************************************************/
#include <rrt_planner/rrt_planner.h>
#include "rrt_planner/rrt.h"
#include "rrt.cpp"
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//register this planner as a BaseGlobalPlanner plugin
//前面是当前namespace::注册的类，后面是基类的namespace::基类的类
//注意，xml用的那个lib是因为cmakelist种的add_lib生成的，一定要严格对应
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planner {

  RRTPlanner::RRTPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }
  

  void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      ros::NodeHandle private_nh("~/" + name);
      initialized_ = true;

      // // 订阅地图,boost::bind最大的作用就是可以在用类内的cb来写sub
      // ros::Subscriber sub = private_nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 10, boost::bind(&RRTPlanner::costmapupdate_callback, this, _1));
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }





  bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();

    if(start.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), start.header.frame_id.c_str());
      return false;
    }

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    //更新地图,这两个还挺重要的
    costmap_ros_->updateMap();
    costmap_=costmap_ros_->getCostmap();


    const double start_yaw = tf2::getYaw(start.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);

    int row_num=costmap_->getSizeInCellsY();
    int col_num=costmap_->getSizeInCellsX();

    // float step_size=(row_num+col_num)/50;
    float step_size=10;
    float end_lim=step_size;
    // float rewire_size=(row_num+col_num)/25;
    float rewire_size=15;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int now_row,now_col,end_row,end_col;
    
    costmap_->worldToMap(start_x, start_y, now_col, now_row);
    costmap_->worldToMap(goal_x, goal_y, end_col, end_row);
    rrt proj(row_num,col_num,step_size,rewire_size,end_lim,now_row,now_col,end_row,end_col,costmap_);
    proj.extend();
    ros::Time plan_time = ros::Time::now();

    for (vector<node>::iterator it = proj.least_path.begin(); it != proj.least_path.end(); it++)
			{
      double world_x, world_y;
      world_x = costmap_->getOriginX() + (it->get_col()) * costmap_->getResolution();
      world_y = costmap_->getOriginY() + (it->get_row()) * costmap_->getResolution();
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = costmap_ros_->getGlobalFrameID();
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
      // std::cout<<plan.back().pose.position.x<<" "<<plan.back().pose.position.y<<endl;
			}
    return (!plan.empty());
  }

};
