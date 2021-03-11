/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/


/*********************************************************************
* layer文件和layered_costmap是相互调用的
* 这个文件更像是layer的上层文件，layer们是作为plugin进来的
* 最重要的是updatemap函数，在里面通过遍历plugins来得到了更新后的costmap
* rolling_window_是一个比较重要的bool参数，会导致除更新区域外的其他地方的数值是否变为default value
**********************************************************************/

#ifndef COSTMAP_2D_LAYERED_COSTMAP_H_
#define COSTMAP_2D_LAYERED_COSTMAP_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <string>

namespace costmap_2d
{
class Layer;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 */
class LayeredCostmap
{
public:
  //初始化参数和一个costmap地图
  //track_unknown: true 地图所有为255; false 为0
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  //遍历所有的plugin的vector，pop点
  ~LayeredCostmap();

  /*
  有三个重要步骤：
  1. 根据rolling_window的true/false来确定是否对costmap的非更新区域赋值为0
  2. 遍历plugin来确定他们的updatebounds
  3. 遍历plugin逐蹭对costmap进行更新
  */
  void updateMap(double robot_x, double robot_y, double robot_yaw);

  //调用costmap_的resizemap函数，本质上就是把原来地图删掉，然后重新建一个地图并覆盖
  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                 bool size_locked = false);

  //API，用于更新的区域
  void getUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  //先调用calculateMinAndMaxDistances来计算机器人内切和外界圆半径（注意，在机器人本体坐标系下，自己的中心永远是0,0）
  //然后调用各个plugin的onfootprintchanged把内切和外界圆半径设置进去
  void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);

  //遍历所有plugin，确定所有地图都是iscurrent
  bool isCurrent();


  /**************************************************************
  从这里开始，就全是给外面的接口了
  **************************************************************/
  //FrameID
  std::string getGlobalFrameID() const
  {
    return global_frame_;
  }
  //Costmap2D的指针
  Costmap2D* getCostmap()
  {
    return &costmap_;
  }
  //是否rolling，static里不rolling，其他都true
  bool isRolling()
  {
    return rolling_window_;
  }
  //是否进行了取反(0和255)
  bool isTrackingUnknown()
  {
    return costmap_.getDefaultValue() == costmap_2d::NO_INFORMATION;
  }
  //plugin的vector的指针
  std::vector<boost::shared_ptr<Layer> >* getPlugins()
  {
    return &plugins_;
  }
  //添加一层plugin
  void addPlugin(boost::shared_ptr<Layer> plugin)
  {
    plugins_.push_back(plugin);
  }
  //是否确定了大小，在resizemap后变成true
  bool isSizeLocked()
  {
    return size_locked_;
  }
  //这个好像是整个地图的四个角？
  void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }
  //updatemap之后变成嗯true
  bool isInitialized()
  {
      return initialized_;
  }
  //最新的footprint
  const std::vector<geometry_msgs::Point>& getFootprint() { return footprint_; }
  //外界圆半径
  double getCircumscribedRadius() { return circumscribed_radius_; }
  //内切原半径
  double getInscribedRadius() { return inscribed_radius_; }

private:
  //全局地图
  Costmap2D costmap_;
  //global fraime id
  std::string global_frame_;
  //是否rolling，除static层之后，应该全是true
  //这是一个比较重要的提数，体现在updateMap中，如果为true，
  //则会使更新区域外的其他地方的数值为0;如果false，不会对costmap对任何处理
  bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot
  //确定各个层的ROS::TIME
  bool current_;
  //update的窗口
  double minx_, miny_, maxx_, maxy_;
  //整个地图的窗口
  unsigned int bx0_, bxn_, by0_, byn_;
  //plugin vector
  std::vector<boost::shared_ptr<Layer> > plugins_;
  //是否updatemap
  bool initialized_;
  //是否resizemap（create map）
  bool size_locked_;
  //外界圆内切圆半径
  double circumscribed_radius_, inscribed_radius_;
  //footprint
  std::vector<geometry_msgs::Point> footprint_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_LAYERED_COSTMAP_H_
