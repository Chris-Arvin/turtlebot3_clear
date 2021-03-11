/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_STATIC_LAYER_H_
#define COSTMAP_2D_STATIC_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>

namespace costmap_2d
{

class StaticLayer : public CostmapLayer
{
public:
  //就初始化了一下reconfiguration
  StaticLayer();
  //reconfiguration的东西
  virtual ~StaticLayer();
  //主要是打开了subscriber，接受了一些和地图相关的参数，调用incoming这个callback来对地图进行了更新
  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  //默认的matchsize是用layer_costmap_来resize了基础的costmap_(调用了costmap的resizemap)
  //这些类层层继承，到了这一层，基本上把以前所有的类/cpp都给包含进去了
  /**
  他的注释写的挺好的，这里先判断了一下这个layer是不是rolling，
  如果它不rolling，则他是static layer，他才配去matchsize;
  如果rolling，则他不配，他只是个局部小地图
  **/
  virtual void matchSize();

private:
  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  //维护的是costmap2d中的costmap_，用new_map去更新它
  //在statc_layer里面，很多东西体现不出来，反而比较繁琐。在其他layer中比较明显
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
  void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
  //reconfiguration的CallBack函数，重新配置了x_ y_ width_ height_这四个参数
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  //输入一个0~100的value，返回的是-1,0,255
  //如果100则根据track_unknown_space来决定返回255/0,若大于100则obstacle(255)，若其他，则free(0)
  //有一个错误处理机制，可能会返回0～255的其他值，但是理论上应该不会出现
  unsigned char interpretValue(unsigned char value);

  //flobal_frame的ID
  std::string global_frame_;
  //接收到的frame的ID
  std::string map_frame_;
  //是否开启一个update的sub，调用incomingupdate这个callback来更新地图
  bool subscribe_to_updates_;
  //是否接收到了地图，在Oninitial中通过sub来更改
  bool map_received_;
  //是否更新完地图数据，在imcoming中被更改
  bool has_updated_data_;
  //
  unsigned int x_, y_, width_, height_;
  //unknwon是被默认为free还是obstacle
  bool track_unknown_space_;
  //是否用max来更新costmap
  bool use_maximum_;
  //该layer是否只更新一次，之后就断开sub（一般只有static layer会为true）
  bool first_map_only_;
  //costmap是否为三元格式
  //默认为：free(0), obstacle(100), unknown(-1)
  bool trinary_costmap_;
  //两个苏北scriber
  ros::Subscriber map_sub_, map_update_sub_;
  //obstacle的值(100)，位置区域的值(-1)
  unsigned char lethal_threshold_, unknown_cost_value_;
  //用于configuration
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_STATIC_LAYER_H_
