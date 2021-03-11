/*********************************************************************
* Authors: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update date:2020.7.15
* 继承自BaseGlobalPlanner，连接move_base文件和规划插件
*********************************************************************/
#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>



namespace rrt_planner{
  //继承自BaseGlobalPlanner，在路径规划算法和move_base文件之间做桥梁
  class RRTPlanner : public nav_core::BaseGlobalPlanner {
    public:
      /**
      * 输入：handle的名字name，全局costmap地图costmap_ros
      * 调用initialize，初始化了两个变量
      */
      RRTPlanner();
      RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * 输入：handle的名字name，全局costmap地图costmap_ros
       * 初始化handle、地图；initialized_=true，表示初始化完成
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * 输入：全局地图下的起点位姿、终点位姿、会被引用赋值的规划的路径
       * 更新地图、设置rrt的参数、调用rrt规划器，对路径规划化得到plan
       */  
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;   //2DROS类型的全局地图
      costmap_2d::Costmap2D* costmap_;    //2D类型的全局地图

      bool initialized_;    //是否被初始化过了
  };
};  
#endif
