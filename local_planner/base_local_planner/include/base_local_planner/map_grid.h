/*********************************************************************
 *Note : 注意，在setlocalgoal()里有一个很关键的地方，我们并不能直接set从起点到终点的所有cost，而是受制于local costmap的大小。
         将传入的global_plan变成了local costmap中离global_plan最近的那个点，这个点是goal，会被用来设置goal_x_和y_。
         我们的local path也是从当前点到这个goal点

 *********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_MAP_GRID_H_
#define TRAJECTORY_ROLLOUT_MAP_GRID_H_

#include <vector>
#include <iostream>
#include <base_local_planner/trajectory_inc.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <base_local_planner/map_cell.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner{
  /**
   * @class MapGrid
   * @brief A grid of MapCell cells that is used to propagate path and goal distances for the trajectory controller.
   * 建一个cell地图，储存每个点到path和goal的距离
   */
  class MapGrid{
    public:
      //初始化地图的大小
      MapGrid();
      MapGrid(unsigned int size_x, unsigned int size_y);
      //copy初始化
      MapGrid(const MapGrid& mg);
      //=初始化
      MapGrid& operator= (const MapGrid& mg);
      ~MapGrid(){}

      //重构（），输入index，返回对应的cell
      inline MapCell& operator() (unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }
      inline MapCell operator() (unsigned int x, unsigned int y) const {
        return map_[size_x_ * y + x];
      }
      //同上
      inline MapCell& getCell(unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }


      //重置map_的所有cell
      void resetPathDist();

      //如果x和y不对应，那么重新初始化一下
      void sizeCheck(unsigned int size_x, unsigned int size_y);

      //初始化map_，每个点的x和y对应的是距离(0,0)的坐标距离，这里没有定义dist
      void commonInit();

      //获得该点的index
      size_t getIndex(int x, int y);

      //将map的作为obstacle的象征
      inline double obstacleCosts() {
        return map_.size();// =x×y
      }

      //map.size()-1 象征某个点还没有到达过
      inline double unreachableCellCosts() {
        return map_.size() + 1;
      }

      //指针更新check_cell，如果其target_dist>current的target_dist+1，则变成后面的那个。有点类似于A*的update周围一圈。
      inline bool updatePathCell(MapCell* current_cell, MapCell* check_cell,
          const costmap_2d::Costmap2D& costmap);

      //根据输入的resolution，在两点间添加中间点。引用赋值global_plan_out
      static void adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
            std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution);

      //dist_queue是一个仅包含path中的cell的vector
      //计算每个local costmap中的每个cell到planned path的最短距离
      //这里是从这个dist_queue出发，往外扩散，遍历整个local costmap，有点广搜的意思
      void computeTargetDistance(std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the local goal point
       * @param goal_queue A queue containing the local goal cell 
       */
      //TODO但是这个函数好像并没有被说明。。
      void computeGoalDistance(std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap);

      //这是在setlocalgoal之后被调用的，给整条路径上所有的点的dist都等于0，然后再调用computetargetdistance
      void setTargetCells(const costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::PoseStamped>& global_plan);

      //给了一个全局的goal，对地图做分辨率处理，对goal做转换处理（制约到local costmap下），仅让goalcell的dist=0，调用computetargetdistance，对所有点进行赋dist处理
      //注意，这里有一个很关键的地方，我们并不能直接set从起点到终点的所有cost，而是受制于local costmap的大小。将传入的global_plan变成了local costmap中离global_plan最近的那个点，这个点是goal，会被用来设置goal_x_和y_
      void setLocalGoal(const costmap_2d::Costmap2D& costmap,
            const std::vector<geometry_msgs::PoseStamped>& global_plan);

      double goal_x_, goal_y_; //global frame中goal的坐标

      unsigned int size_x_, size_y_; //local costmap的尺寸

    private:

      std::vector<MapCell> map_; ///< @brief Storage for the MapCells

  };
};

#endif
