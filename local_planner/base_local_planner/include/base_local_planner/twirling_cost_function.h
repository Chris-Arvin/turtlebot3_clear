/*********************************************************************
 * Author: Morgan Quigley
 *********************************************************************/

#ifndef TWIRLING_COST_FUNCTION_H
#define TWIRLING_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner {

/**
 * This class provides a cost based on how much a robot "twirls" on its
 * way to the goal. With differential-drive robots, there isn't a choice,
 * but with holonomic or near-holonomic robots, sometimes a robot spins
 * more than you'd like on its way to a goal. This class provides a way
 * to assign a penalty purely to rotational velocities.
 */
class TwirlingCostFunction: public base_local_planner::TrajectoryCostFunction {
public:

  TwirlingCostFunction() {}
  ~TwirlingCostFunction() {}
  //返回角速度的绝对值，把角速度大小作为评分？？
  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};
};

} /* namespace base_local_planner */
#endif /* TWIRLING_COST_FUNCTION_H_ */
