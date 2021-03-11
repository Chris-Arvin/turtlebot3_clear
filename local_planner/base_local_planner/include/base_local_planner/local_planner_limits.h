/***********************************************************
* 设定机器人的动力参数
 ***********************************************************/


#ifndef __base_local_planner__LOCALPLANNERLIMITS_H__
#define __base_local_planner__LOCALPLANNERLIMITS_H__

#include <Eigen/Core>

namespace base_local_planner
{
class LocalPlannerLimits
{
public:

  double max_vel_trans;
  double min_vel_trans;
  double max_vel_x;
  double min_vel_x;
  double max_vel_y;
  double min_vel_y;
  double max_vel_theta;
  double min_vel_theta;
  double acc_lim_x;
  double acc_lim_y;
  double acc_lim_theta;
  double acc_lim_trans;
  bool   prune_plan;
  double xy_goal_tolerance;
  double yaw_goal_tolerance;
  double trans_stopped_vel;
  double theta_stopped_vel;
  bool   restore_defaults;

  LocalPlannerLimits() {}

  LocalPlannerLimits(
      double nmax_vel_trans,
      double nmin_vel_trans,
      double nmax_vel_x,
      double nmin_vel_x,
      double nmax_vel_y,
      double nmin_vel_y,
      double nmax_vel_theta,
      double nmin_vel_theta,
      double nacc_lim_x,
      double nacc_lim_y,
      double nacc_lim_theta,
      double nacc_lim_trans,
      double nxy_goal_tolerance,
      double nyaw_goal_tolerance,
      bool   nprune_plan = true,
      double ntrans_stopped_vel = 0.1,
      double ntheta_stopped_vel = 0.1):
        max_vel_trans(nmax_vel_trans),
        min_vel_trans(nmin_vel_trans),
        max_vel_x(nmax_vel_x),
        min_vel_x(nmin_vel_x),
        max_vel_y(nmax_vel_y),
        min_vel_y(nmin_vel_y),
        max_vel_theta(nmax_vel_theta),
        min_vel_theta(nmin_vel_theta),
        acc_lim_x(nacc_lim_x),
        acc_lim_y(nacc_lim_y),
        acc_lim_theta(nacc_lim_theta),
        acc_lim_trans(nacc_lim_trans),
        prune_plan(nprune_plan),
        xy_goal_tolerance(nxy_goal_tolerance),
        yaw_goal_tolerance(nyaw_goal_tolerance),
        trans_stopped_vel(ntrans_stopped_vel),
        theta_stopped_vel(ntheta_stopped_vel) {}

  ~LocalPlannerLimits() {}

  /**
   * @brief  Get the acceleration limits of the robot
   * @return  The acceleration limits of the robot
   */
  Eigen::Vector3f getAccLimits() {
    Eigen::Vector3f acc_limits;
    acc_limits[0] = acc_lim_x;
    acc_limits[1] = acc_lim_y;
    acc_limits[2] = acc_lim_theta;
    return acc_limits;
  }

};

}
#endif // __LOCALPLANNERLIMITS_H__
