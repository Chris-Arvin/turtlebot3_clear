/*********************************************************************
* Author: Eitan Marder-Eppstein
* Note: 没太看到这个.h
*********************************************************************/
#ifndef POINT_GRID_H_
#define POINT_GRID_H_

#include <vector>
#include <list>
#include <cfloat>
#include <geometry_msgs/Point.h>
#include <costmap_2d/observation.h>
#include <base_local_planner/world_model.h>

#include <sensor_msgs/PointCloud2.h>

namespace base_local_planner {
  /**
   * @class PointGrid
   * @brief A class that implements the WorldModel interface to provide
   * free-space collision checks for the trajectory controller. This class
   * stores points binned into a grid and performs point-in-polygon checks when
   * necessary to determine the legality of a footprint at a given
   * position/orientation.
   */
  class PointGrid : public WorldModel {
    public:
      //point转换成grid
      //max_z是z的最大高度。obstacle_range是障碍物探测的最大距离。min_separation是points的最大距离
      PointGrid(double width, double height, double resolution, geometry_msgs::Point origin, 
          double max_z, double obstacle_range, double min_separation);

      /**
       * @brief  Destructor for a point grid
       */
      virtual ~PointGrid(){}

      //给一个长方形的左下和右上角点，引用赋值points，填充式push矩形内所有点
      void getPointsInRange(const geometry_msgs::Point& lower_left, const geometry_msgs::Point& upper_right, std::vector< std::list<geometry_msgs::Point32>* >& points);


      //检测
      virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius);

      using WorldModel::footprintCost;

      /**
       * @brief  Inserts observations from sensors into the point grid
       * @param footprint The footprint of the robot in its current location
       * @param observations The observations from various sensors 
       * @param laser_scans The laser scans used to clear freespace (the point grid only uses the first scan which is assumed to be the base laser)
       */
      void updateWorld(const std::vector<geometry_msgs::Point>& footprint, 
          const std::vector<costmap_2d::Observation>& observations, const std::vector<PlanarLaserScan>& laser_scans);

      //从world到local map的转换，引用赋值
      inline bool gridCoords(geometry_msgs::Point pt, unsigned int& gx, unsigned int& gy) const 
      {
        if(pt.x < origin_.x || pt.y < origin_.y){
          gx = 0;
          gy = 0;
          return false;
        }
        gx = (int) ((pt.x - origin_.x)/resolution_);
        gy = (int) ((pt.y - origin_.y)/resolution_);

        if(gx >= width_ || gy >= height_){
          gx = 0;
          gy = 0;
          return false;
        }

        return true;
      }

      //world到local map的转换，多态
      inline bool gridCoords(const geometry_msgs::Point32& pt, unsigned int& gx, unsigned int& gy) const 
      {
        if(pt.x < origin_.x || pt.y < origin_.y){
          gx = 0;
          gy = 0;
          return false;
        }
        gx = (int) ((pt.x - origin_.x)/resolution_);
        gy = (int) ((pt.y - origin_.y)/resolution_);

        if(gx >= width_ || gy >= height_){
          gx = 0;
          gy = 0;
          return false;
        }

        return true;
      }

      //local map到world的转换
      inline void getCellBounds(unsigned int gx, unsigned int gy, geometry_msgs::Point& lower_left, geometry_msgs::Point& upper_right) const 
      {
        lower_left.x = gx * resolution_ + origin_.x;
        lower_left.y = gy * resolution_ + origin_.y;

        upper_right.x = lower_left.x + resolution_;
        upper_right.y = lower_left.y + resolution_;
      }


      //2D距离
      inline double sq_distance(const geometry_msgs::Point32& pt1, const geometry_msgs::Point32& pt2)
      {
        return (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y);
      }



      //获得index
      inline unsigned int gridIndex(unsigned int gx, unsigned int gy) const {
        return(gx + gy * width_);
      }

      /**
       * @brief  Check the orientation of a pt c with respect to the vector a->b
       * @param a The start point of the vector 
       * @param b The end point of the vector 
       * @param c The point to compute orientation for
       * @return orient(a, b, c) < 0 ----> Right, orient(a, b, c) > 0 ----> Left 
       */
      //todo 没看懂
      inline double orient(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point32& c)
      {
        double acx = a.x - c.x;
        double bcx = b.x - c.x;
        double acy = a.y - c.y;
        double bcy = b.y - c.y;
        return acx * bcy - acy * bcx;
      }

      /**
       * @brief  Check the orientation of a pt c with respect to the vector a->b
       * @param a The start point of the vector 
       * @param b The end point of the vector 
       * @param c The point to compute orientation for
       * @return orient(a, b, c) < 0 ----> Right, orient(a, b, c) > 0 ----> Left 
       */
      template<typename T>
      inline double orient(const T& a, const T& b, const T& c){
        double acx = a.x - c.x;
        double bcx = b.x - c.x;
        double acy = a.y - c.y;
        double bcy = b.y - c.y;
        return acx * bcy - acy * bcx;
      }

      /**
       * @brief  Check if two line segmenst intersect
       * @param v1 The first point of the first segment 
       * @param v2 The second point of the first segment 
       * @param u1 The first point of the second segment 
       * @param u2 The second point of the second segment 
       * @return True if the segments intersect, false otherwise
       */
      //检测两个先是否相交
      inline bool segIntersect(const geometry_msgs::Point32& v1, const geometry_msgs::Point32& v2,
          const geometry_msgs::Point32& u1, const geometry_msgs::Point32& u2){
        return (orient(v1, v2, u1) * orient(v1, v2, u2) < 0) && (orient(u1, u2, v1) * orient(u1, u2, v2) < 0);
      }

      /**
       * @brief  Find the intersection point of two lines
       * @param v1 The first point of the first segment 
       * @param v2 The second point of the first segment 
       * @param u1 The first point of the second segment 
       * @param u2 The second point of the second segment 
       * @param result The point to be filled in
       */
      //找两个线的相交点
      void intersectionPoint(const geometry_msgs::Point& v1, const geometry_msgs::Point& v2, 
          const geometry_msgs::Point& u1, const geometry_msgs::Point& u2, 
          geometry_msgs::Point& result);

      /**
       * @brief  Check if a point is in a polygon
       * @param pt The point to be checked 
       * @param poly The polygon to check against
       * @return True if the point is in the polygon, false otherwise
       */
      //判断一个点是否在一堆point中（矩形），如果点在polygon中，返回true
      bool ptInPolygon(const geometry_msgs::Point32& pt, const std::vector<geometry_msgs::Point>& poly);

      //把ptpush到cells_到
      void insert(const geometry_msgs::Point32& pt);

      /**
       * @brief  Find the distance between a point and its nearest neighbor in the grid
       * @param pt The point used for comparison 
       * @return  The distance between the point passed in and its nearest neighbor in the point grid
       */
      double nearestNeighborDistance(const geometry_msgs::Point32& pt);

      /**
       * @brief  Find the distance between a point and its nearest neighbor in a cell
       * @param pt The point used for comparison 
       * @param gx The x coordinate of the cell
       * @param gy The y coordinate of the cell
       * @return  The distance between the point passed in and its nearest neighbor in the cell
       */
      double getNearestInCell(const geometry_msgs::Point32& pt, unsigned int gx, unsigned int gy);

      /**
       * @brief  Removes points from the grid that lie within the polygon
       * @param poly A specification of the polygon to clear from the grid 
       */
      void removePointsInPolygon(const std::vector<geometry_msgs::Point> poly);

      /**
       * @brief  Removes points from the grid that lie within a laser scan
       * @param  laser_scan A specification of the laser scan to use for clearing
       */
      void removePointsInScanBoundry(const PlanarLaserScan& laser_scan);

      /**
       * @brief  Checks to see if a point is within a laser scan specification
       * @param  pt The point to check
       * @param  laser_scan The specification of the scan to check against
       * @return True if the point is contained within the scan, false otherwise
       */
      bool ptInScan(const geometry_msgs::Point32& pt, const PlanarLaserScan& laser_scan);

      /**
       * @brief  Get the points in the point grid
       * @param  cloud The point cloud to insert the points into
       */
      void getPoints(sensor_msgs::PointCloud2& cloud);

    private:
      double resolution_; ///< @brief The resolution of the grid in meters/cell
      geometry_msgs::Point origin_; ///< @brief The origin point of the grid
      unsigned int width_; ///< @brief The width of the grid in cells
      unsigned int height_; ///< @brief The height of the grid in cells
      std::vector< std::list<geometry_msgs::Point32> > cells_; ///< @brief Storage for the cells in the grid
      double max_z_;  ///< @brief The height cutoff for adding points as obstacles
      double sq_obstacle_range_;  ///< @brief The square distance at which we no longer add obstacles to the grid
      double sq_min_separation_;  ///< @brief The minimum square distance required between points in the grid
      std::vector< std::list<geometry_msgs::Point32>* > points_;  ///< @brief The lists of points returned by a range search, made a member to save on memory allocation
  };
};
#endif
