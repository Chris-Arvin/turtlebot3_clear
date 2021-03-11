/*********************************************************************
* Authors: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* 之前的bug出现在：没有讲initialized_设置为true，整个initilaize函数都没做。。
*********************************************************************/
#include <global_py_api/global_py_api.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <py_flag.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(api_planner::APIPlanner, nav_core::BaseGlobalPlanner)

namespace api_planner {

  APIPlanner::APIPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  APIPlanner::APIPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }

  void APIPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    name_ = name;
    //确定障碍物映射函数，把0~255映射到0~100
    cost_translation_table_ = new char[256];
    // special values:
    cost_translation_table_[0] = 0;  // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle 内切障碍物
    cost_translation_table_[254] = 100;  // LETHAL obstacle 致命的障碍物
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++)
    {
      cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
    }

    initialized_ = true;    //initialized完成
  }

  /**
  * 对地图做预处理，把costmap转换成occupancygrid的形式，用于后期发送client
  */
  void APIPlanner::prepareGrid()
  {
    double resolution = costmap_->getResolution();

    grid_.header.frame_id = "map";  //默认的global_frame
    grid_.header.stamp = ros::Time::now();
    grid_.info.resolution = resolution;

    grid_.info.width = costmap_->getSizeInCellsX();
    grid_.info.height = costmap_->getSizeInCellsY();
    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid_.info.origin.position.x = wx - resolution / 2; //虽然我也不知道为啥用-resolution/2，但是，这样不报错，直接写0，会有问题。。本质上还是有一个偏移的感觉
    grid_.info.origin.position.y = wy - resolution / 2; 
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;

    grid_.data.resize(grid_.info.width * grid_.info.height);
    unsigned char* data = costmap_->getCharMap();
    for (unsigned int i = 0; i < grid_.data.size(); i++)
    {
      grid_.data[i] = cost_translation_table_[ data[ i ]];
    }
  }

  bool APIPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
 {
    //整理grid_
    prepareGrid();
    ros::NodeHandle private_nh;
    //初始化client发布器和内容
    python_client= private_nh.serviceClient<global_py_api::py_flag>("global_path_py");  
    global_py_api::py_flag srv;
    srv.request.map=grid_;  //地图
    srv.request.start=start;  //起点
    srv.request.goal=goal;  //终点
    //call server（python文件）
    if(python_client.call(srv))
    {
      plan_= srv.response.path.poses;
      ros::Time plan_time = ros::Time::now();
      if(!plan_.empty())
      {
        //赋值plan
        for (std::vector<geometry_msgs::PoseStamped>::iterator it = plan_.begin(); it != plan_.end(); it++)
        {
          double world_x, world_y;
          //todo 根据地图映射来说，这里需不需要加或减一个resolution/2？！！
          world_x = costmap_->getOriginX() + it->pose.position.x * costmap_->getResolution()+costmap_->getResolution()/2,0;
          world_y = costmap_->getOriginY() + it->pose.position.y * costmap_->getResolution()+costmap_->getResolution()/2,0;
          geometry_msgs::PoseStamped pose;
          pose.header.stamp = plan_time;
          pose.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose.pose.position.x = world_x;
          pose.pose.position.y = world_y;
          pose.pose.position.z = 0.0;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
          // std::cout<<world_x<<","<<world_y<<std::endl;
          plan.push_back(pose);
          // std::cout<<plan.back().pose.position.x<<" "<<plan.back().pose.position.y<<endl;
        }
        return (!plan.empty());
      }
    }
    return 0;
  }


};
