/*********************************************************************
* Author: Eitan Marder-Eppstein
* Developer: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* ROS Navigation整个包的一个命名规则是，带有ROS后缀的类完成的是该子过程与整体和其他过程的衔接框架和数据流通，不带ROS后缀的类中完成该部分的实际工作，并作为带有ROS后缀的类的成员。
* 本质上是调用了navfn的dijkstra方法，但是允许goal进行微小偏移。详情解析见navfn_ros  --2020.7.7
*********************************************************************/
#ifndef NAVFN_NAVFN_ROS_H_
#define NAVFN_NAVFN_ROS_H_

#include <ros/ros.h>
#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <navfn/potarr_point.h>

namespace navfn {
  /**
   * @class NavfnROS
   * @brief Provides a ROS wrapper for the navfn planner which runs a fast, interpolated navigation function on a costmap.
   */
  class NavfnROS : public nav_core::BaseGlobalPlanner {
    public:

      //初始化节点名称，全局地图costmap，全局规划的坐标系名称frameID
      NavfnROS();
      NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      NavfnROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

      //被初始化调用
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

      //本质上就是，先调用了computePotential，再调用getPlanFromPotential，用引用赋值来得到plan
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

      //设置起点是（0，0），目标点是转换到map下的world_point，调用planner_->calcNavFnDijkstra()用dijkstra计算路径，planner给的参数默认为false。。
      //本质上，这里是严格起点和终点，然后进行规划。
      bool computePotential(const geometry_msgs::Point& world_point);

      //在调用它之前，需要先调用computePotential，来保证可以严格出来一条规划路径。
      //在makeplan的最后面被调用，输出一个goal，服用赋值plan。核心是planner_->calcPath(costmap_->getSizeInCellsX() * 4);
      //它和computePotential的根本区别在于，它允许goal进行偏移，以保障得到一条更好的路径（实际上这个偏移 要的是一定范围内离goal最近的点，就很迷。。）
      bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      
      //看某个world中的点能否映射到map的一个点
      double getPointPotential(const geometry_msgs::Point& world_point);
      //输入一个world地图下的点，分别在x和y方向在tolerance_距离内进行偏移，调用getPointPotential看这个点能不能映射到map上
      bool validPointPotential(const geometry_msgs::Point& world_point);
      bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

      ~NavfnROS(){}

      bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

      /**
       * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
       */
      costmap_2d::Costmap2D* costmap_;  //全局costmap
      boost::shared_ptr<NavFn> planner_;  //navfn的规划器
      ros::Publisher plan_pub_; //发布路径
      ros::Publisher potarr_pub_; //发布点云图
      bool initialized_;  //是否被初始化
      bool allow_unknown_;  //是否允许障碍物未知区域的存在
      bool visualize_potential_;  //是否发布2D点云图


    private:
      //计算两点间距离
      inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
        double dx = p1.pose.position.x - p2.pose.position.x;
        double dy = p1.pose.position.y - p2.pose.position.y;
        return dx*dx +dy*dy;
      }
      //引用赋值，map到world的转换
      void mapToWorld(double mx, double my, double& wx, double& wy);
      //清空（mx，my）这个点，global_pose没有用到
      void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
      double planner_window_x_, planner_window_y_;
      double default_tolerance_;  //默认的可偏移距离，默认是0
      boost::mutex mutex_;
      ros::ServiceServer make_plan_srv_;  //make_plan的service
      std::string global_frame_;  //frameID
  };
};

#endif
