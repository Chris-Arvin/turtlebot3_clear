/*********************************************************************
* Developer：Arvin
* Email: 1711506@mail.nankai.edu.cn
* Data: 2020.7.7
*********************************************************************/

#include <navfn/navfn_ros.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>

namespace cm=costmap_2d;
namespace rm=geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

namespace navfn {

class NavfnWithCostmap : public NavfnROS
{
public:
  NavfnWithCostmap(string name, Costmap2DROS* cmap);
  bool makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp);

private:
  void poseCallback(const rm::PoseStamped::ConstPtr& goal);
  Costmap2DROS* cmap_;
  ros::ServiceServer make_plan_service_;
  ros::Subscriber pose_sub_;
};


bool NavfnWithCostmap::makePlanService(MakeNavPlan::Request& req, MakeNavPlan::Response& resp)
{
  vector<PoseStamped> path;

  req.start.header.frame_id = "map";
  req.goal.header.frame_id = "map";
  bool success = makePlan(req.start, req.goal, path);
  resp.plan_found = success;
  if (success) {
    resp.path = path;
  }

  return true;
}

void NavfnWithCostmap::poseCallback(const rm::PoseStamped::ConstPtr& goal) {
  geometry_msgs::PoseStamped global_pose;
  cmap_->getRobotPose(global_pose);
  vector<PoseStamped> path;
  makePlan(global_pose, *goal, path);
}


NavfnWithCostmap::NavfnWithCostmap(string name, Costmap2DROS* cmap) : 
  NavfnROS(name, cmap)
{
  ros::NodeHandle private_nh("~");
  cmap_ = cmap;
  make_plan_service_ = private_nh.advertiseService("make_plan", &NavfnWithCostmap::makePlanService, this);
  pose_sub_ = private_nh.subscribe<rm::PoseStamped>("goal", 1, &NavfnWithCostmap::poseCallback, this);
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  costmap_2d::Costmap2DROS lcr("costmap", buffer);

  navfn::NavfnWithCostmap navfn("navfn_planner", &lcr);

  ros::spin();
  return 0;
}
















