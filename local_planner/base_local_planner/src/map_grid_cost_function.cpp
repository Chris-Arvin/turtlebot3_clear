/*********************************************************************
 * Author: TKruse
 * Notation: Arvin
 * E-mail: 1711506@mail.nankai.edu.cn
 *********************************************************************/

#include <base_local_planner/map_grid_cost_function.h>

namespace base_local_planner {

MapGridCostFunction::MapGridCostFunction(costmap_2d::Costmap2D* costmap,
    double xshift,
    double yshift,
    bool is_local_goal_function,
    CostAggregationType aggregationType) :
    costmap_(costmap),
    map_(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()),
    aggregationType_(aggregationType),
    xshift_(xshift),
    yshift_(yshift),
    is_local_goal_function_(is_local_goal_function),
    stop_on_failure_(true) {}

void MapGridCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;
}

bool MapGridCostFunction::prepare() {
  map_.resetPathDist();

  if (is_local_goal_function_) {
    map_.setLocalGoal(*costmap_, target_poses_);
  } else 
  {
    map_.setTargetCells(*costmap_, target_poses_);
  }
  return true;
}

double MapGridCostFunction::getCellCosts(unsigned int px, unsigned int py) 
{
  double grid_dist = map_(px, py).target_dist;
  return grid_dist;
}

double MapGridCostFunction::scoreTrajectory(Trajectory &traj) 
{
  double cost = 0.0;
  if (aggregationType_ == Product) 
  {
    cost = 1.0;
  }
  double px, py, pth;
  unsigned int cell_x, cell_y;
  double grid_dist;

  //遍历trajectory的所有点
  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) 
  {
    traj.getPoint(i, px, py, pth);

    // translate point forward if specified
    if (xshift_ != 0.0) 
    {
      px = px + xshift_ * cos(pth);
      py = py + xshift_ * sin(pth);
    }
    // translate point sideways if specified
    if (yshift_ != 0.0) 
    {
      px = px + yshift_ * cos(pth + M_PI_2);
      py = py + yshift_ * sin(pth + M_PI_2);
    }

    //点落在了costmap外面，错误返回（但理论上，不应该出现这个错误）
    if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) 
    {
      //we're off the map
      ROS_WARN("Off Map %f, %f", px, py);
      return -4.0;
    }
    grid_dist = getCellCosts(cell_x, cell_y);
    //某个cell是障碍物或超出local costmap的大小，错误返回
    if (stop_on_failure_) {
      if (grid_dist == map_.obstacleCosts()) 
      {
        return -3.0;
      }
      else if (grid_dist == map_.unreachableCellCosts()) 
      {
        return -2.0;
      }
    }

    //三种情况
    switch( aggregationType_ ) 
    {
    case Last:
      cost = grid_dist;
      break;
    case Sum:
      cost += grid_dist;
      break;
    case Product:
      if (cost > 0) 
      {
        cost *= grid_dist;
      }
      break;
    }
  }
  return cost;
}

} /* namespace base_local_planner */
