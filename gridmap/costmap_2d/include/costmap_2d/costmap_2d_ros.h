/*********************************************************************
* Author: Eitan Marder-Eppstein
*         David V. Lu!!
* Developer：Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update date：2020.7.16
*********************************************************************/


/*********************************************************************
* 重要参数
  1. stop_：与start、pause、stop三个函数相关，是start的开启开关
  2. stop_updates_：是updateMap的开关，只有flase时，才会开启updateMap()
*********************************************************************/


/*********************************************************************
* 逻辑梳理
  1. 核心流程：初始化，然后调用void mapUpdateLoop(double frequency)，循环做：
                    在里面调用updatemap()，
                      在里面调用layered_costmap_->updatemap()进行更新
  2. 外界管理：通过调用start(), pause(), stop()来管理地图更新
  3. 他这个插件的active()这种好像也是有自己的一个线程的，通过外界管理来启动或关闭这个线程。在navigation里好像并没有用到1.部分的流程。。
*********************************************************************/


#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.hpp>
#include <tf2/LinearMath/Transform.h>

class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */
class Costmap2DROS
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(const std::string &name, tf2_ros::Buffer& tf);
  ~Costmap2DROS();

  /**
   * 开始更新
   * 遍历所有的layered_costmap的插件，分别调用activate()函数来进行激活
   * stopped_=false, stop_updates_=false
   */
  void start();

  /**
   * 停止更新
   * 遍历调用插件的deactivate
   * stopped_=true, stop_updates_=true, initialized_=false
   */
  void stop();

  /**
   * 暂停更新
   * stop_updates_=true, initilized_=false
   */
  void pause();

  /**
   * 重启地图更新？
   * stop_updates_=false
   */
  void resume();

  /**
  * 调用layered_costmap_->updatemap来更新地图
  */
  void updateMap();

  /**
   * 重置layers
   * 遍历所有plugin，分别调用reset()
   */
  void resetLayers();

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent()
    {
      return layered_costmap_->isCurrent();
    }

  /**
   * 输入：会被引用赋值的global_pose
   * 获取机器人当前位姿，引用赋值
   * todo我一直不太明白这个是怎么实现的。。好像是在tf_这个buffer里保存了一些坐标系间的转换矩阵，
      getIdentity本质上是针对全局坐标系的处理，对robot_pose用只是为了对他做初始化？。。
      用tf_.transform，结合第一个参数的frame_id和第三个参数做处理，给第二个参数引用赋值？
   */
  bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const;

  /** 
  * 一些列对外的借口
  */
  std::string getName() const
    {
      return name_;
    }
  double getTransformTolerance() const
    {
      return transform_tolerance_;
    }
  Costmap2D* getCostmap()
    {
      return layered_costmap_->getCostmap();  //注意，这个是char型的costmap
    }
  std::string getGlobalFrameID()
    {
      return global_frame_;
    }
  std::string getBaseFrameID()
    {
      return robot_base_frame_;
    }
  LayeredCostmap* getLayeredCostmap()
    {
      return layered_costmap_;
    }
  geometry_msgs::Polygon getRobotFootprintPolygon()
  {
    return costmap_2d::toPolygon(padded_footprint_);
  }
  std::vector<geometry_msgs::Point> getRobotFootprint()
  {
    return padded_footprint_;
  }

  std::vector<geometry_msgs::Point> getUnpaddedRobotFootprint()
  {
    return unpadded_footprint_;
  }

  /**
   * 输入：被引用赋值的机器人当前位姿下的footprint
   * 获取机器人当前位姿，把padded footprint转换到当前位姿下，引用赋值
   */
  void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;

  /** 
   * 输入：被转化成point格式的机器人轮廓点集points
   * 先调用padFoorprint来填充轮廓，然后设置到layered_costmap的footprint种
   */
  void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points);

  /*
  * 输入：sub到的机器人轮廓footprint
  * 调用setunpaddedrobotfootprint，设置layered_costmap的机器人轮廓
  */
  void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint);

protected:
  LayeredCostmap* layered_costmap_;   //建立layered costmap
  std::string name_;     //插件名称
  tf2_ros::Buffer& tf_;  //用于transform的buffer
  std::string global_frame_;  //全局坐标系的ID
  std::string robot_base_frame_;  //机器人坐标系的ID
  double transform_tolerance_;  ///< timeout before transform errors

private:
  /** 
  * 读取configuration参数
  */
  void readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                               const costmap_2d::Costmap2DConfig &old_config);
  //加载4个旧的layer插件参数
  void loadOldParameters(ros::NodeHandle& nh);
  //加载新参数的警告
  void warnForOldParameters(ros::NodeHandle& nh);
  void checkOldParam(ros::NodeHandle& nh, const std::string &param_name);
  /**
  * 输入：插件名称，插件类型，handle名字
  * 复制保存参数
  */
  void copyParentParameters(const std::string& plugin_name, const std::string& plugin_type, ros::NodeHandle& nh);
  void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
  /**
  * 给定当前时间，确定机器人是否停止，赋值old_pose_=机器人当前位姿
  */
  void movementCB(const ros::TimerEvent &event);
  /**
  * 循环调用updatemap()来更新地图
  */
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_;   //是否关闭地图更新线程
  bool stop_updates_, initialized_, stopped_, robot_stopped_;
  boost::thread* map_update_thread_;  ///< @brief A thread for updating the map
  ros::Timer timer_;
  ros::Time last_publish_;  //上一次publish的时间
  ros::Duration publish_cycle;  //publish的周期
  pluginlib::ClassLoader<Layer> plugin_loader_;   //插件加载器
  geometry_msgs::PoseStamped old_pose_;
  Costmap2DPublisher* publisher_;   //costmap发布器
  dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;    //重新配置的server

  boost::recursive_mutex configuration_mutex_;    //重新配置的线程

  ros::Subscriber footprint_sub_;   //接收footprint
  ros::Publisher footprint_pub_;    //发布footprint
  std::vector<geometry_msgs::Point> unpadded_footprint_;  //没有被填充的footprint
  std::vector<geometry_msgs::Point> padded_footprint_;    //被填充后的footprint
  float footprint_padding_; //padding值（外膨胀值），默认0
  costmap_2d::Costmap2DConfig old_config_;
};
// class Costmap2DROS
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_ROS_H
