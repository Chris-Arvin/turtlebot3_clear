/*********************************************************************
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_VOXEL_WORLD_MODEL_H_
#define TRAJECTORY_ROLLOUT_VOXEL_WORLD_MODEL_H_
#include <vector>
#include <list>
#include <cfloat>
#include <geometry_msgs/Point.h>
#include <costmap_2d/observation.h>
#include <base_local_planner/world_model.h>

//voxel grid stuff
#include <voxel_grid/voxel_grid.h>

namespace base_local_planner {
  /**
   * @class VoxelGridModel
   * @brief A class that implements the WorldModel interface to provide grid
   * based collision checks for the trajectory controller using a 3D voxel grid.
   */
  class VoxelGridModel : public WorldModel {
    public:
      /**
       * @param  max_z The maximum height for an obstacle to be added to the grid
       * @param  obstacle_range The maximum distance for obstacles to be added to the grid
       */
      VoxelGridModel(double size_x, double size_y, double size_z, double xy_resolution, double z_resolution,
          double origin_x, double origin_y, double origin_z, double max_z, double obstacle_range);

      /**
       * @brief  Destructor for the world model
       */
      virtual ~VoxelGridModel(){}

      /**
       * @brief  Checks if any obstacles in the voxel grid lie inside a convex footprint that is rasterized into the grid
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return Positive if all the points lie outside the footprint, negative otherwise
       */
      virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

      using WorldModel::footprintCost;

      //将observation添加到obstacle_grid_中，将laser_scans扫到的地方清空
      void updateWorld(const std::vector<geometry_msgs::Point>& footprint,
          const std::vector<costmap_2d::Observation>& observations, const std::vector<PlanarLaserScan>& laser_scans);

      //获得所有的点
      void getPoints(sensor_msgs::PointCloud2& cloud);

    private:
      //计算一个line的cost，这个cost是线中cost最大的point的cost
      double lineCost(int x0, int x1, int y0, int y1);

      //如果(x,y)是障碍物，返回-1，traj无效；否则返回1
      double pointCost(int x, int y);
      
      //激光扫描到的非障碍物地图，会被一条线一条线地清空（线的起点是当前位置，线的终点是激光或地图的劲头），针对obstacle_grid_操作
      //todo这里面有一个维度变换的操作，我不太清楚
      void removePointsInScanBoundry(const PlanarLaserScan& laser_scan, double raytrace_range);

      inline bool worldToMap3D(double wx, double wy, double wz, unsigned int& mx, unsigned int& my, unsigned int& mz){
        if(wx < origin_x_ || wy < origin_y_ || wz < origin_z_)
          return false;
        mx = (int) ((wx - origin_x_) / xy_resolution_);
        my = (int) ((wy - origin_y_) / xy_resolution_);
        mz = (int) ((wz - origin_z_) / z_resolution_);
        return true;
      }

      inline bool worldToMap2D(double wx, double wy, unsigned int& mx, unsigned int& my){
        if(wx < origin_x_ || wy < origin_y_)
          return false;
        mx = (int) ((wx - origin_x_) / xy_resolution_);
        my = (int) ((wy - origin_y_) / xy_resolution_);
        return true;
      }

      inline void mapToWorld3D(unsigned int mx, unsigned int my, unsigned int mz, double& wx, double& wy, double& wz){
        //returns the center point of the cell
        wx = origin_x_ + (mx + 0.5) * xy_resolution_;
        wy = origin_y_ + (my + 0.5) * xy_resolution_;
        wz = origin_z_ + (mz + 0.5) * z_resolution_;
      }

      inline void mapToWorld2D(unsigned int mx, unsigned int my, double& wx, double& wy){
        //returns the center point of the cell
        wx = origin_x_ + (mx + 0.5) * xy_resolution_;
        wy = origin_y_ + (my + 0.5) * xy_resolution_;
      }

      inline double dist(double x0, double y0, double z0, double x1, double y1, double z1){
        return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
      }

      //把pt添加到obstacle_grid_中
      inline void insert(const geometry_msgs::Point32& pt){
        unsigned int cell_x, cell_y, cell_z;
        if(!worldToMap3D(pt.x, pt.y, pt.z, cell_x, cell_y, cell_z))
          return;
        obstacle_grid_.markVoxel(cell_x, cell_y, cell_z);
      }

      voxel_grid::VoxelGrid obstacle_grid_;
      double xy_resolution_;
      double z_resolution_;
      double origin_x_;
      double origin_y_;
      double origin_z_;
      double max_z_;  ///< @brief The height cutoff for adding points as obstacles
      double sq_obstacle_range_;  ///< @brief The square distance at which we no longer add obstacles to the grid

  };
};
#endif
