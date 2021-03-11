/*********************************************************************
 * Author: TKruse
 *********************************************************************/

#ifndef OBSTACLE_COST_FUNCTION_H_
#define OBSTACLE_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>

namespace base_local_planner {

//检测robotfootprint和costmap，只要存在撞击，则这条路径直接负分，被抛弃
//（TODO有一个问题，costmap被膨胀后，为啥还要检测footprint呢，图啥啊）
class ObstacleCostFunction : public TrajectoryCostFunction {

public:
  ObstacleCostFunction(costmap_2d::Costmap2D* costmap);
  ~ObstacleCostFunction();
  //返回prepare_
  bool prepare();
  //遍历路径，调用footprint对每个点进行打分，根据sum_scores_判断是sum还是max评分
  double scoreTrajectory(Trajectory &traj);
  //设置分数
  void setSumScores(bool score_sums){ sum_scores_=score_sums; }
  //设置参数
  void setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed);
  //设置固定的footprint_spec
  void setFootprint(std::vector<geometry_msgs::Point> footprint_spec);

  //traj是轨迹；scaling_speed是一个用来比较的速度，如果当前速度(sqrt(vx^2+vy^2))大于它，则利用max_trans_vel来进行一个缩减，保证速度是小鱼max_scaling_factor的
  static double getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor);
  //TODO调用了world_model的footprintcost来对当前footprint的障碍物碰撞继续打分？
  static double footprintCost(
      const double& x,
      const double& y,
      const double& th,
      double scale,
      std::vector<geometry_msgs::Point> footprint_spec,
      costmap_2d::Costmap2D* costmap,
      base_local_planner::WorldModel* world_model);

private:
  costmap_2d::Costmap2D* costmap_;
  std::vector<geometry_msgs::Point> footprint_spec_;
  base_local_planner::WorldModel* world_model_;
  double max_trans_vel_;
  bool sum_scores_;
  //footprint scaling with velocity;
  double max_scaling_factor_, scaling_speed_;
};

} /* namespace base_local_planner */
#endif /* OBSTACLE_COST_FUNCTION_H_ */
