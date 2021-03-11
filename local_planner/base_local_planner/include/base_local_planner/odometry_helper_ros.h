/*********************************************************************
 * Author: TKruse
 * Note: boost::mutex::scoped_lock，要多看一下这个函数，多线程的知识。在ros还挺常见的。
 *********************************************************************/

#ifndef ODOMETRY_HELPER_ROS2_H_
#define ODOMETRY_HELPER_ROS2_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner {

class OdometryHelperRos {
public:

  //调用setOdomTopic()，设置subscriber
  OdometryHelperRos(std::string odom_topic = "");
  ~OdometryHelperRos() {}

  //设置base_odom_
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  //引用赋值
  void getOdom(nav_msgs::Odometry& base_odom);
  //用base_odom_来设置robot_vel
  void getRobotVel(geometry_msgs::PoseStamped& robot_vel);

  //设置odom_sub_这个subscriber，用来接收odom_topic 
  void setOdomTopic(std::string odom_topic);

  /** @brief Return the current odometry topic. */
  std::string getOdomTopic() const { return odom_topic_; }

private:

  std::string odom_topic_;  //odom的topic
  ros::Subscriber odom_sub_;  //odom的sub
  nav_msgs::Odometry base_odom_;  //base_odom_，我们自己的里程计
  boost::mutex odom_mutex_; //线程
  std::string frame_id_; //frame id
};

} /* namespace base_local_planner */
#define CHUNKY 1
#endif /* ODOMETRY_HELPER_ROS2_H_ */
