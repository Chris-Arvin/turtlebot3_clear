/*********************************************************************
 * 
 *********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_MAP_CELL_H_
#define TRAJECTORY_ROLLOUT_MAP_CELL_H_

#include <base_local_planner/trajectory_inc.h>

namespace base_local_planner {
  //针对一个小的cell处理
  /*
  * target_dist:距离规划路径的最短距离
  * target_mark该点是否是目标点
  * within_robot是否在机器人的footprint里
  */
  class MapCell{
    public:

      MapCell();
      //copy初始化
      MapCell(const MapCell& mc);

      unsigned int cx, cy; ///< @brief Cell index in the grid map
      double target_dist; ///< @brief Distance to planner's path
      bool target_mark; ///< @brief Marks for computing path/goal distances
      bool within_robot; ///< @brief Mark for cells within the robot footprint
  };
};

#endif
