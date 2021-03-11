/*********************************************************************
* Author: Eitan Marder-Eppstein
* 对local costmap和global costmap分别处理。遍历所有layer，一层一层地调用costmap_2d自己的函数进行清空。
*********************************************************************/
#ifndef CLEAR_COSTMAP_RECOVERY_H_
#define CLEAR_COSTMAP_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>

namespace clear_costmap_recovery{
  /**
   * @class ClearCostmapRecovery
   * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
   */
  class ClearCostmapRecovery : public nav_core::RecoveryBehavior {
    public:

      ClearCostmapRecovery();

      /**
       * 输入为设置的handle的名字，_， 全局地图， 局部地图；
       * 在内部确定了更新哪个地图（global，local）（默认both），确定了更新哪个层（默认obstacle）。
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

      //更新affected_maps_进行了地图更新：先clear，再强制update
      void runBehavior();

    private:
      //输入为整个地图，注意是指针
      //在内部遍历每个layer，分别调用clearmap函数
      void clear(costmap_2d::Costmap2DROS* costmap);

      //输入为一个costmap（实际上是一个layer）和机器人当前位置，调用costmap_2d的函数对其进行清空
      void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y);

      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_; //全局地图，局部地图
      std::string name_;  //设置handle的名字
      tf2_ros::Buffer* tf_; //其实我一直我不太知道这个是干啥的。。
      bool initialized_;  //是否初始化完成
      bool force_updating_; //清空costmap后，强制更新一下（不等move_base调用update线程了，自己直接来call）
      double reset_distance_; //重置的范围，以机器人为中心，上下左右各延伸reset_distance_/2
      std::string affected_maps_; //只清空局部地图，还是只全局，还是两个一起清空
      std::set<std::string> clearable_layers_; //即将被clear的layer的名字
  };
};
#endif
