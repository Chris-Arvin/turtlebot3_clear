/*********************************************************************
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/obstacle_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace base_local_planner {

ObstacleCostFunction::ObstacleCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) 
{
  if (costmap != NULL) 
  {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
  }
}

ObstacleCostFunction::~ObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}


void ObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) 
{
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

void ObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) 
{
  footprint_spec_ = footprint_spec;
}

bool ObstacleCostFunction::prepare() 
{
  return true;
}

double ObstacleCostFunction::scoreTrajectory(Trajectory &traj) 
{
  double cost = 0;
  //防止超速
  double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
  double px, py, pth;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) 
  {
    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth,
        scale, footprint_spec_,
        costmap_, world_model_);

    if(f_cost < 0)
    {
        return f_cost;
    }

    if(sum_scores_)
        cost +=  f_cost;
    else
        cost = std::max(cost, f_cost);
  }
  return cost;
}

double ObstacleCostFunction::getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) 
{
  double vmag = hypot(traj.xv_, traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  //如果速度太大了，进行了比例缩减，起码要限制在max_scaling_factor里面
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

double ObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    double scale,
    std::vector<geometry_msgs::Point> footprint_spec,
    costmap_2d::Costmap2D* costmap,
    base_local_planner::WorldModel* world_model) 
{

  //check if the footprint is legal
  // TODO: Cache inscribed radius
  double footprint_cost = world_model->footprintCost(x, y, th, footprint_spec);

  if (footprint_cost < 0) 
  {
    return -6.0;
  }
  unsigned int cell_x, cell_y;

  //we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) 
  {
    return -7.0;
  }

  double occ_cost = std::max(std::max(0.0, footprint_cost), double(costmap->getCost(cell_x, cell_y)));

  return occ_cost;
}

} /* namespace base_local_planner */
