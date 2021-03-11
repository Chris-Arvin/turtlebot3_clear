/*********************************************************************
* Author: Eitan Marder-Eppstein
* Developer: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
*********************************************************************/


#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_H_

#include <vector>
#include <Eigen/Core>


#include <dwa_local_planner/DWAPlannerConfig.h>

//for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>

#include <nav_msgs/Path.h>

namespace dwa_local_planner {
  /**
   * @class DWAPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class DWAPlanner {
    public:
      //接收参数并初始化了一个评分器，评分器里面push了多个cost
      DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

      //
      void reconfigure(DWAPlannerConfig &cfg);

      //利用generator_生成路径然后打分，如果大于0，则不碰障碍物
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);

      //本质上是调用了scored_sampling_planner_的findbestpath
      base_local_planner::Trajectory findBestPath(
          const geometry_msgs::PoseStamped& global_pose,
          const geometry_msgs::PoseStamped& global_vel,
          geometry_msgs::PoseStamped& drive_velocities);

      //更新路径，重新生成cost
      void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
          const std::vector<geometry_msgs::PoseStamped>& new_plan,
          const std::vector<geometry_msgs::Point>& footprint_spec);

      //仿真时间（走一步的时间）
      double getSimPeriod() { return sim_period_; }

      //返回path goal occ的加权和
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      //把global_plan给planner_util_
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    private:

      base_local_planner::LocalPlannerUtil *planner_util_;

      double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
      double path_distance_bias_, goal_distance_bias_, occdist_scale_;
      Eigen::Vector3f vsamples_;

      double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
      base_local_planner::Trajectory result_traj_;

      double forward_point_distance_;

      std::vector<geometry_msgs::PoseStamped> global_plan_;

      boost::mutex configuration_mutex_;
      std::string frame_id_;
      ros::Publisher traj_cloud_pub_;
      bool publish_cost_grid_pc_; ///< @brief Whether or not to build and publish a PointCloud
      bool publish_traj_pc_;

      double cheat_factor_;

      base_local_planner::MapGridVisualizer map_viz_; //用于可视化

      // see constructor body for explanations
      base_local_planner::SimpleTrajectoryGenerator generator_; //生成器
      base_local_planner::OscillationCostFunction oscillation_costs_;   //震动cost
      base_local_planner::ObstacleCostFunction obstacle_costs_; //障碍物cost
      base_local_planner::MapGridCostFunction path_costs_;  //

      base_local_planner::MapGridCostFunction goal_costs_;  //所有路径的cost
      base_local_planner::MapGridCostFunction goal_front_costs_; //所有路径+局部目标前向探测的cost
      base_local_planner::MapGridCostFunction alignment_costs_; //
      base_local_planner::TwirlingCostFunction twirling_costs_; //所有/当前角度的绝对值

      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  };
};
#endif
