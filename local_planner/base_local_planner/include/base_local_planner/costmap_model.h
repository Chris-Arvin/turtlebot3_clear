/*********************************************************************
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_COSTMAP_MODEL_
#define TRAJECTORY_ROLLOUT_COSTMAP_MODEL_

#include <base_local_planner/world_model.h>
// For obstacle data access
#include <costmap_2d/costmap_2d.h>

namespace base_local_planner {
  /**
   * @class CostmapModel
   * @brief A class that implements the WorldModel interface to provide grid
   * based collision checks for the trajectory controller using the costmap.
   */
  class CostmapModel : public WorldModel {
    public:
      /**
       * @brief  Constructor for the CostmapModel
       * @param costmap The costmap that should be used
       * @return
       */
      CostmapModel(const costmap_2d::Costmap2D& costmap);

      /**
       * @brief  Destructor for the world model
       */
      virtual ~CostmapModel(){}
      using WorldModel::footprintCost;

      //遍历机器人每两个点，求两点连线的最小cost，取所有cost的max，返回
      virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

      //取0到1这条先中所有点中cost最小的那个
      double lineCost(int x0, int x1, int y0, int y1) const;

      //返回x，y点的cost
      double pointCost(int x, int y) const;

    private:
      const costmap_2d::Costmap2D& costmap_; ///< @brief Allows access of costmap obstacle information

  };
};
#endif
