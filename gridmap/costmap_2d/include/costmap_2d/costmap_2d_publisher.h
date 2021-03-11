/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/


/*
  coding note:
  重写别人的代码，可以学到很多思维和函数的用法


*/

#ifndef COSTMAP_2D_COSTMAP_2D_PUBLISHER_H_
#define COSTMAP_2D_COSTMAP_2D_PUBLISHER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

namespace costmap_2d
{

class Costmap2DPublisher
{
public:
  /**
    初始化参数
    最重要的是初始化了cost_translation_table，把0～255映射到0～100。索引方式：c...[0~255]，返回0~100
  */
  Costmap2DPublisher(ros::NodeHandle * ros_node, Costmap2D* costmap, std::string global_frame,
                     std::string topic_name, bool always_send_full_costmap = false);
  //空的
  ~Costmap2DPublisher();

  //扩大的趋势更新bounds
  void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn)
  {
    x0_ = std::min(x0, x0_);
    xn_ = std::max(xn, xn_);
    y0_ = std::min(y0, y0_);
    yn_ = std::max(yn, yn_);
  }

  /**
   * @brief  Publishes the visualization data over ROS
   */
  void publishCostmap();

  bool active()
  {
    return active_;
  }

private:
  //把costmap转换成grid的格式，用于发布，注意这个grid是一维的，数值0～100
  void prepareGrid();

  //先调用prepareGrid然后publish到初始化Costmap2DPublisher时赋名字的topic中
  void onNewSubscription(const ros::SingleSubscriberPublisher& pub);

  ros::NodeHandle* node;
  Costmap2D* costmap_;
  std::string global_frame_;
  unsigned int x0_, xn_, y0_, yn_;
  double saved_origin_x_, saved_origin_y_;
  bool active_;
  bool always_send_full_costmap_;
  ros::Publisher costmap_pub_;
  ros::Publisher costmap_update_pub_;
  nav_msgs::OccupancyGrid grid_;
  static char* cost_translation_table_;  ///< Translate from 0-255 values in costmap to -1 to 100 values in message.
};
}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_2D_PUBLISHER_H
