/*********************************************************************
* Author: Eitan Marder-Eppstein
* 所以其实插件机制啥的，和通过头文件来调用然后写一个继承，是一样的呀。。。
  本质上是通过虚函数来限制函数形式，做到整齐一点（接口统一）
* 两个makeplan，一个initialize
* 理论上，BaseGlobalPlanner是不可能被实例化的（初始化函数是protected的），都是通过被继承来起作用的
*********************************************************************/
#ifndef NAV_CORE_BASE_GLOBAL_PLANNER_H
#define NAV_CORE_BASE_GLOBAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace nav_core {
  /**
   * @class BaseGlobalPlanner
   * @brief Provides an interface for global planners used in navigation. All global planners written as plugins for the navigation stack must adhere to this interface.
   */
  class BaseGlobalPlanner{
    public:
      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) = 0;

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @param cost The plans calculated cost
       * @return True if a valid plan was found, false otherwise
       */
      virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                            const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan,
                            double& cost)
      {
        cost = 0;
        return makePlan(start, goal, plan);
      }

      /**
       * @brief  Initialization function for the BaseGlobalPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~BaseGlobalPlanner(){}

    protected:
      BaseGlobalPlanner(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_BASE_GLOBAL_PLANNER_H