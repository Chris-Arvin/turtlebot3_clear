/*********************************************************************
 * Author: David V. Lu!!
 *********************************************************************/

/********************************************************************
整个函数里，基本上没有定义啥东西，除了赋值，都是虚汗数。可以看作是那几个具体plugin的模板
他一一直在和layered_costmap相互调用
*********************************************************************/

#ifndef COSTMAP_2D_LAYER_H_
#define COSTMAP_2D_LAYER_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <string>
#include <tf2_ros/buffer.h>

namespace costmap_2d
{
class LayeredCostmap;

class Layer
{
public:
  /*************************
  除了下面这三个函数，别的都是virtual，都没有默认定义
  *************************/
  //啥也没干，就赋值了
  Layer();
  //初始化+调用onInitialize。在里面设置的parent layer是干啥的？。。
  void initialize(LayeredCostmap* parent, std::string name, tf2_ros::Buffer *tf);
  //从父亲节处获得footprint
  const std::vector<geometry_msgs::Point>& getFootprint() const;

  /*************************
  以下全是虚函数
  *************************/

  /**
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   *
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y) {}

  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {}

  /** @brief Stop publishers. */
  virtual void deactivate() {}

  /** @brief Restart publishers if they've been stopped. */
  virtual void activate() {}

  virtual void reset() {}

  virtual ~Layer() {}

  /**
   * @brief Check to make sure all the data in the layer is up to date.
   *        If the layer is not up to date, then it may be unsafe to
   *        plan using the data from this layer, and the planner may
   *        need to know.
   *
   *        A layer's current state should be managed by the protected
   *        variable current_.
   * @return Whether the data in the layer is up to date.
   */
  bool isCurrent() const
  {
    return current_;
  }

  /** @brief Implement this to make this layer match the size of the parent costmap. */
  virtual void matchSize() {}

  std::string getName() const
  {
    return name_;
  }

  /** @brief LayeredCostmap calls this whenever the footprint there
   * changes (via LayeredCostmap::setFootprint()).  Override to be
   * notified of changes to the robot's footprint. */
  virtual void onFootprintChanged() {}

protected:
  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   *
   * tf_, name_, and layered_costmap_ will all be set already when this is called. */
  virtual void onInitialize() {}

  LayeredCostmap* layered_costmap_;
  bool current_;
  bool enabled_;  ///< Currently this var is managed by subclasses. TODO: make this managed by this class and/or container class.
  std::string name_;
  tf2_ros::Buffer *tf_;

private:
  std::vector<geometry_msgs::Point> footprint_spec_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_LAYER_H_
