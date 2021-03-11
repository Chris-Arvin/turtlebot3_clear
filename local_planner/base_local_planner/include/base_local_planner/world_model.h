/*********************************************************************
* Author: Eitan Marder-Eppstein
* Note: 本质上就是先把机器人的footprint转换到全局坐标系下，再和costmap作对比，看机器人是否在地图内/压到障碍物或未知区域
*********************************************************************/
#ifndef TRAJECTORY_ROLLOUT_WORLD_MODEL_H_
#define TRAJECTORY_ROLLOUT_WORLD_MODEL_H_

#include <vector>
#include <costmap_2d/observation.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Point.h>
#include <base_local_planner/planar_laser_scan.h>

namespace base_local_planner 
{

  //world和controller交互
  class WorldModel{
    public:
      //检测机器人当前是否合法地在world中。
      /**
       * @return Positive if all the points lie outside the footprint, negative otherwise:
       *          -1 if footprint covers at least a lethal obstacle cell, or
       *          -2 if footprint covers at least a no-information cell, or
       *          -3 if footprint is partially or totally outside of the map
       */
      //footprint的cost的评价函数，这是frame
      virtual double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius) = 0;
      
      //输入的x，y和theta是当前的机器人坐标和朝向
      double footprintCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius = 0.0, double circumscribed_radius=0.0)
      {
        double cos_th = cos(theta);
        double sin_th = sin(theta);
        std::vector<geometry_msgs::Point> oriented_footprint;
        //把机器人的foorprint变成全局坐标下的坐标
        for(unsigned int i = 0; i < footprint_spec.size(); ++i)
        {
          geometry_msgs::Point new_pt;
          new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
          new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
          oriented_footprint.push_back(new_pt);
        }

        geometry_msgs::Point robot_position;
        robot_position.x = x;
        robot_position.y = y;

        if(inscribed_radius==0.0)
        {
          //引用赋值，去算inscribed和circumscribed的radius
          costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
        }
        //调用真正的footprintcost进行评价
        return footprintCost(robot_position, oriented_footprint, inscribed_radius, circumscribed_radius);
      }

      //这里的多态是为了防止有憨批多输一个没用的数
      double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
          double inscribed_radius, double circumscribed_radius, double extra) {
        return footprintCost(position, footprint, inscribed_radius, circumscribed_radius);
      }

      virtual ~WorldModel(){}

    protected:
      WorldModel(){}
  };

};
#endif
