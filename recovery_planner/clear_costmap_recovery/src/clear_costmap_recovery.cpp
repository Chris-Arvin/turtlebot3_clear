/*********************************************************************
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace clear_costmap_recovery {
ClearCostmapRecovery::ClearCostmapRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {}

void ClearCostmapRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
  if(!initialized_)
  {
    //初始化和地图相关的内容
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //接收参数
    ros::NodeHandle private_nh("~/" + name_);
    private_nh.param("reset_distance", reset_distance_, 3.0);
    private_nh.param("force_updating", force_updating_, false);
    private_nh.param("affected_maps", affected_maps_, std::string("both"));

    //如果affected_maps_不正确的话，就globa和local一起更新
    if (affected_maps_ != "local" && affected_maps_ != "global" && affected_maps_ != "both")
    {
      ROS_WARN("Wrong value for affected_maps parameter: '%s'; valid values are 'local', 'global' or 'both'; " \
               "defaulting to 'both'", affected_maps_.c_str());
      affected_maps_ = "both";
    }

    //默认是更新obstacle层，如果没有从launch中接收到，那么clearbel_layers=default的
    std::vector<std::string> clearable_layers_default, clearable_layers;
    clearable_layers_default.push_back( std::string("obstacles") );
    private_nh.param("layer_names", clearable_layers, clearable_layers_default);
    //赋值给类的私有变量
    for(unsigned i=0; i < clearable_layers.size(); i++) 
    {
        ROS_INFO("Recovery behavior will clear layer '%s'", clearable_layers[i].c_str());
        clearable_layers_.insert(clearable_layers[i]);
    }
    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void ClearCostmapRecovery::runBehavior()
{
  //确定初始化没问题
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }
  //确定地图不是空的
  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  ROS_WARN("Clearing %s costmap%s to unstuck robot (%.2fm).", affected_maps_.c_str(),
           affected_maps_ == "both" ? "s" : "", reset_distance_);
  
  //清空全局地图并强制更新
  ros::WallTime t0 = ros::WallTime::now();
  if (affected_maps_ == "global" || affected_maps_ == "both")
  {
    clear(global_costmap_);

    if (force_updating_)
      global_costmap_->updateMap();

    ROS_DEBUG("Global costmap cleared in %fs", (ros::WallTime::now() - t0).toSec());
  }
  //清空局部地图并强制更新
  t0 = ros::WallTime::now();
  if (affected_maps_ == "local" || affected_maps_ == "both")
  {
    clear(local_costmap_);

    if (force_updating_)
      local_costmap_->updateMap();

    ROS_DEBUG("Local costmap cleared in %fs", (ros::WallTime::now() - t0).toSec());
  }
}


void ClearCostmapRecovery::clear(costmap_2d::Costmap2DROS* costmap)
{
  //获取多个layer的plugin
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  //获得机器人当前位姿
  geometry_msgs::PoseStamped pose;
  if(!costmap->getRobotPose(pose))
  {
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;

  //遍历插件，调用clearmap
  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) 
  {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    std::string name = plugin->getName();
    int slash = name.rfind('/');
    if( slash != std::string::npos )
    {
        name = name.substr(slash+1);
    }

    if(clearable_layers_.count(name)!=0)
    {
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y);
    }
  }
}


void ClearCostmapRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap,
                                        double pose_x, double pose_y)
{
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  //四个角
  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

  //world2map
  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);
  
  //清0
  costmap->clearArea(start_x, start_y, end_x, end_y);

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
  return;
}

};
