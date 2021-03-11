/*********************************************************************
* Author: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update: 2020.7.16
* 路径规划部分，提供了python的借口
*********************************************************************/
#ifndef API_PLANNER_H_
#define API_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <vector>
#include <global_py_api/py_flag.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>

namespace api_planner{
  /**
   * @class CarrotPlanner
   * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
   */
  class APIPlanner : public nav_core::BaseGlobalPlanner 
  {
    public:


      APIPlanner();
      APIPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      //做规划，主要是和python通信
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      
      //从costmap_2d_publisher复制过来的，把costmap2D变成occopyGrid格式，用于发送client
      void prepareGrid();

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      std::string name_; //handle的名字
      costmap_2d::Costmap2D* costmap_;  //地图
      ros::ServiceClient python_client; //发布规划指令
      bool initialized_;  //是否已经被初始化过了（规范接口）
      std::vector<geometry_msgs::PoseStamped> plan_;  //python脚本规划出来的路径
      nav_msgs::OccupancyGrid grid_;  //地图
      char* cost_translation_table_;  ///< Translate from 0-255 values in costmap to -1 to 100 values in message.
  };
};  
#endif
