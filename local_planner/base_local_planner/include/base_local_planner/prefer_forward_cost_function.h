/*********************************************************************
 * Author: TKruse
 *********************************************************************/

#ifndef PREFER_FORWARD_COST_FUNCTION_H_
#define PREFER_FORWARD_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner 
{

class PreferForwardCostFunction: public base_local_planner::TrajectoryCostFunction 
{
public:

  PreferForwardCostFunction(double penalty) : penalty_(penalty) {}
  ~PreferForwardCostFunction() {}
  //对一个轨迹进行评分，这里主要是从机器人前向移动来判断的
  //如果机器人方向移动或者低速旋转前进，这是不好的，返回penality_；否则返回角速度绝对值*10，是个不错的轨迹
  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};

  void setPenalty(double penalty) {
    penalty_ = penalty;
  }

private:
  double penalty_;  //惩罚
};

} /* namespace base_local_planner */
#endif /* PREFER_FORWARD_COST_FUNCTION_H_ */
