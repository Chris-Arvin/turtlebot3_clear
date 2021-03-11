/*********************************************************************
 * Author: TKruse
 * Note: 震动cost。如果目标的flag和当前的状态相反，则打负分。
 *********************************************************************/

#ifndef OSCILLATION_COST_FUNCTION_H_
#define OSCILLATION_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <Eigen/Core>

namespace base_local_planner {

class OscillationCostFunction: public base_local_planner::TrajectoryCostFunction 
{
public:
  OscillationCostFunction();
  virtual ~OscillationCostFunction();

  //输入一个轨迹，如果某种flag和traj的当前状态相反，则返回一个负值（例如flag让前进，而vx是负的）
  double scoreTrajectory(Trajectory &traj);

  bool prepare() {return true;};

  //flag全false
  void resetOscillationFlags();

  //更新flag
  void updateOscillationFlags(Eigen::Vector3f pos, base_local_planner::Trajectory* traj, double min_vel_trans);
  //设置参数
  void setOscillationResetDist(double dist, double angle);

private:

  //比较前一状态和当前状态，如果差的比较多，调用reset，falg全变成false
  void resetOscillationFlagsIfPossible(const Eigen::Vector3f& pos, const Eigen::Vector3f& prev);

  //设置flag，如果产生了flag更改，则返回true
  bool setOscillationFlags(base_local_planner::Trajectory* t, double min_vel_trans);

  // flags
  bool strafe_pos_only_, strafe_neg_only_, strafing_pos_, strafing_neg_;
  bool rot_pos_only_, rot_neg_only_, rotating_pos_, rotating_neg_;
  bool forward_pos_only_, forward_neg_only_, forward_pos_, forward_neg_;

  // param
  double oscillation_reset_dist_, oscillation_reset_angle_;

  Eigen::Vector3f prev_stationary_pos_;
};

} /* namespace base_local_planner */
#endif /* OSCILLATION_COST_FUNCTION_H_ */
