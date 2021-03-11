/*********************************************************************
 * Author: TKruse
 *********************************************************************/

#ifndef MAP_GRID_COST_FUNCTION_H_
#define MAP_GRID_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/map_grid.h>

namespace base_local_planner {

//对mapgried评分，可以从三个角度：上一个point，所有point的总和，所有非0点的总和，在scoreTrajectory中被调用
enum CostAggregationType { Last, Sum, Product};

/**
 * This class provides cost based on a map_grid of a small area of the world.
 * The map_grid covers a the costmap, the costmap containing the information
 * about sensed obstacles. The map_grid is used by setting
 * certain cells to distance 0, and then propagating distances around them,
 * filling up the area reachable around them.
 *
 * The approach using grid_maps is used for computational efficiency, allowing to
 * score hundreds of trajectories very quickly.
 *
 * This can be used to favor trajectories which stay on a given path, or which
 * approach a given goal.
 * @param costmap_ros Reference to object giving updates of obstacles around robot
 * @param xshift where the scoring point is with respect to robot center pose
 * @param yshift where the scoring point is with respect to robot center pose
 * @param is_local_goal_function, scores for local goal rather than whole path
 * @param aggregationType how to combine costs along trajectory
 */
class MapGridCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  MapGridCostFunction(costmap_2d::Costmap2D* costmap,
      double xshift = 0.0,
      double yshift = 0.0,
      bool is_local_goal_function = false,
      CostAggregationType aggregationType = Last);

  ~MapGridCostFunction() {}

  //初始化一些东西
  void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);
  void setXShift(double xshift) {xshift_ = xshift;}
  void setYShift(double yshift) {yshift_ = yshift;}

  //默认是true，如果是true，那么在这个path上如果出现问题，则整个path被抛弃
  void setStopOnFailure(bool stop_on_failure) {stop_on_failure_ = stop_on_failure;}

  //初始化距离，根据is_local_goal_function_判断是只把goal的dist设置为0，还是整个path都是0
  bool prepare();

  //遍历trajectory，对他进行打分，共有三种方式。
  double scoreTrajectory(Trajectory &traj);

  //障碍物cell的cost
  double obstacleCosts() {
    return map_.obstacleCosts();
  }

  //到达不了的cell的cost（=map_.size()-1）
  double unreachableCellCosts() {
    return map_.unreachableCellCosts();
  }

  //获得某个点的dist
  double getCellCosts(unsigned int cx, unsigned int cy);

private:
  std::vector<geometry_msgs::PoseStamped> target_poses_;
  costmap_2d::Costmap2D* costmap_;

  base_local_planner::MapGrid map_;
  CostAggregationType aggregationType_;
  /// xshift and yshift allow scoring for different
  // points of robots than center, like fron or back
  // this can help with alignment or keeping specific
  // wheels on tracks both default to 0
  double xshift_; //这个shift多少有点没意义。。默认是0，默认挺好的。。
  double yshift_;
  bool is_local_goal_function_; //如果true，则仅对goal的dist降为0；否则整个path都是0
  bool stop_on_failure_;
};

} /* namespace base_local_planner */
#endif /* MAP_GRID_COST_FUNCTION_H_ */
