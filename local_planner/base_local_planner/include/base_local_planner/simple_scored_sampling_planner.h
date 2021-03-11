/*********************************************************************
 * Author: TKruse
 *********************************************************************/

#ifndef SIMPLE_SCORED_SAMPLING_PLANNER_H_
#define SIMPLE_SCORED_SAMPLING_PLANNER_H_

#include <vector>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>

namespace base_local_planner {

/**
 * @class SimpleScoredSamplingPlanner
 * @brief Generates a local plan using the given generator and cost functions.
 * Assumes less cost are best, and negative costs indicate infinite costs
 *
 * This is supposed to be a simple and robust implementation of
 * the TrajectorySearch interface. More efficient search may well be
 * possible using search heuristics, parallel search, etc.
 */
class SimpleScoredSamplingPlanner : public base_local_planner::TrajectorySearch {
public:

  ~SimpleScoredSamplingPlanner() {}

  SimpleScoredSamplingPlanner() {}

  //做初始化
  SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples = -1);

  //对一条traj进行打分（cost），如果打分超出了best_traj_cost直接退出，返回best_traj_cost
  double scoreTrajectory(Trajectory& traj, double best_traj_cost);

  //输入的这俩都是需要被赋值的。。
  //traj是best traj，每个点的保存格式是(x,y,yaw)
  //all_explored是所有可行的traj，保存每个轨迹的完整信息
  //核心函数是调用gen_->nextTrajectory和scoreTrajectory
  bool findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored = 0);


private:
  std::vector<TrajectorySampleGenerator*> gen_list_;  //路径生成器
  std::vector<TrajectoryCostFunction*> critics_;  //打分器

  int max_samples_; //最大路径生成次数
};




} // namespace

#endif /* SIMPLE_SCORED_SAMPLING_PLANNER_H_ */
