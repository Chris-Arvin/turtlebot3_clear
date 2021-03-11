/*********************************************************************
* Author: Eitan Marder-Eppstein
* 原理同之前
* 这里是当机器人长期不动或陷入某种死循环时被调用的，具体的还没看
*********************************************************************/
#ifndef NAV_CORE_RECOVERY_BEHAVIOR_H
#define NAV_CORE_RECOVERY_BEHAVIOR_H

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>

namespace nav_core {
  /**
   * @class RecoveryBehavior
   * @brief Provides an interface for recovery behaviors used in navigation. All recovery behaviors written as plugins for the navigation stack must adhere to this interface.
   */
  class RecoveryBehavior{
    public:
      /**
       * @brief  Initialization function for the RecoveryBehavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 
       */
      virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap) = 0;

      /**
       * @brief   Runs the RecoveryBehavior
       */
      virtual void runBehavior() = 0;

      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~RecoveryBehavior(){}

    protected:
      RecoveryBehavior(){}
  };
};  // namespace nav_core

#endif  // NAV_CORE_RECOVERY_BEHAVIOR_H
