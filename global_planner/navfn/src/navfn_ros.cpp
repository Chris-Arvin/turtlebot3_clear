/*********************************************************************
* Author: Eitan Marder-Eppstein
* Developer: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* 这是global_planner的前身，这里面的A*不能用，只能用dijkstra，但是效果比global_planner要好的多，而且global_planner有bug！
* global_planner是把多个文件拆开了，这里面是混到一起了。
*********************************************************************/

/*********************************************************************
* 和其他几个文件的关系
  nav_ros: 本质上就是planner_core，可以被navfn_node调用，可以去掉用navfn的planner。启承上起下的作用。
  navfn_node: 从该文件启动本文件
  navfn: planner_的具体实现，真正进行规划
  potarr_point: 定义了点云数据的struct
**********************************************************************/

/*********************************************************************
Update Log:
* 添加注释，和planner_core进行对比      ------2020.5.9
**********************************************************************/



#include <navfn/navfn_ros.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/point_cloud2_iterator.h>

//注册插件navfn/NavfnROS
PLUGINLIB_EXPORT_CLASS(navfn::NavfnROS, nav_core::BaseGlobalPlanner)

namespace navfn 
{

  //三个重载的初始化。主要是怕憨憨没分清CostMap2D和CostMap2DROS
  //最主要的是进行了initialize
  NavfnROS::NavfnROS() : 
    costmap_(NULL),  
    planner_(), 
    initialized_(false), 
    allow_unknown_(true) 
    {}

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros): 
    costmap_(NULL),  
    planner_(), 
    initialized_(false), 
    allow_unknown_(true) 
    {
      initialize(name, costmap_ros);
    }

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame): 
    costmap_(NULL),  
    planner_(), 
    initialized_(false), 
    allow_unknown_(true) 
    {
      initialize(name, costmap, global_frame);
    }

  //这个cpp里重载了2个initialize，在move_base里调用和实际调用的，都是下面这个
  //初始化了多个参数和发布器
  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
  {
    //如果已经初始化过了，直接返回
    if(initialized_)
      {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        return;
      }
    
    /*正确进行初始化*/

    //对NavFn类进行实例化，传入参数为地图大小
    costmap_ = costmap;
    global_frame_ = global_frame;
    planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));
    
    //初始化全局param
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("visualize_potential", visualize_potential_, false);
    private_nh.param("allow_unknown", allow_unknown_, true);
    private_nh.param("planner_window_x", planner_window_x_, 0.0);
    private_nh.param("planner_window_y", planner_window_y_, 0.0);
    private_nh.param("default_tolerance", default_tolerance_, 0.0);
    
    //发布潜在路径，默认是false
    if(visualize_potential_)
      potarr_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("potential", 1);
    //发布plan给gazebo
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    //被make_plan call
    make_plan_srv_ =  private_nh.advertiseService("make_plan", &NavfnROS::makePlanService, this);

    //flag，标志初始化完成
    initialized_ = true;      
  }

  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }


  //这个函数，输入是一个world_point（原始点）和一个tolerance
  //这里面，以x-tolerance和y-tolerance为左上角起点，往右下搜索，
  //调用getPointPotential来看该点是否可以映射到map中一个free点
  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point, double tolerance)
  {
    //还未初始化，exit(1)
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //获得resolution和该点
    double resolution = costmap_->getResolution();
    geometry_msgs::Point p;
    p = world_point;

    //分别针对x和y，在两侧偏移tolerance的基础上，判断该点是否是潜在点
    p.y = world_point.y - tolerance;
    while(p.y <= world_point.y + tolerance)
    {
      p.x = world_point.x - tolerance;
      while(p.x <= world_point.x + tolerance)
      {
        double potential = getPointPotential(p);
        if(potential < POT_HIGH)
        {
          return true;
        }
        p.x += resolution;
      }
      p.y += resolution;
    }

    return false;
  }
  //重载
  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point)
  {
    return validPointPotential(world_point, default_tolerance_);
  }

  //看某个world中的点能否映射到map的一个free点
  double NavfnROS::getPointPotential(const geometry_msgs::Point& world_point)
  {
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return -1.0;
    }
    unsigned int mx, my;
    //不能从world转到map，直接错误，DBL_MAX默认大于POT_HIGH?
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;
    //能转换，根据planner_自己的数组，返回一个potential值
    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
  }


  //这里应该是真正的在调用planner_中的函数在做规划
  //设置地图，设置起终点，用dijkstra进行规划
  bool NavfnROS::computePotential(const geometry_msgs::Point& world_point)
  {
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //planner(NavFn类)用的地图是一个数组，而不是costmap
    //这里是在设置planner_的array
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    //把0,0作为起点，输入的world转化的map点作为目标点，进行设置？
    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;
    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;
    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    //用dijkstra进行规划，返回是否规划成功
    return planner_->calcNavFnDijkstra();
  }

  //把一个点mx和my设置为free
  void NavfnROS::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my)
  {
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  //提供给外界的接口，用req给的start和goal，利用makePlan规划路径，返回到resp
  bool NavfnROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
  {
    makePlan(req.start, req.goal, resp.plan.poses);
    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;
    return true;
  } 

  //map到world坐标的转换，引用赋值
  void NavfnROS::mapToWorld(double mx, double my, double& wx, double& wy) 
  {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  //重载，用default_tolerance_(默认是0.0即不忍受任何坐标误差，从param中接收)去调用makePlan
  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }

  //这个是实际用的makePlan，前半部分和computePotential基本一样
  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    //初始化规划线程
    boost::mutex::scoped_lock lock(mutex_);
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //清空之前的plan，错误检测，确保收到的目标和当前位姿都是基于当前的global fram
    plan.clear();
    ros::NodeHandle n;
    if(goal.header.frame_id != global_frame_)
    {
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }
    if(start.header.frame_id != global_frame_)
    {
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), start.header.frame_id.c_str());
      return false;
    }

    //start坐标转换
    double wx = start.pose.position.x;
    double wy = start.pose.position.y;
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my))
    {
      ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }
    //start点肯定不是obstacle，清空
    clearRobotCell(start, mx, my);
    //确保planner_的地图Array设置正确
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);
    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    //goal坐标转换
    wx = goal.pose.position.x;
    wy = goal.pose.position.y;
    //如果goal转换失败的话，就设置成原点
    if(!costmap_->worldToMap(wx, wy, mx, my))
    {
      //如果不能忍受任何的坐标误差的话，是不可能的，直接错误退出（默认的tolerance会导致直接退出）
      if(tolerance <= 0.0)
      {
        ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
      }
      mx = 0;
      my = 0;
    }
    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    //设置planner_的start和goal
    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    //默认是用dijkstra算的
    //bool success = planner_->calcNavFnAstar();
    planner_->calcNavFnDijkstra(true);

    /************************************************************************************
    截止到现在，和computePotential基本完全一样
    *************************************************************************************/

    //这一部分是针对global形态的goal作处理
    //遍历goal+-tolerance处的像素点，看将他转化到map下时，作为那个点更合适
    //这里把best_pose作为了最适合goal的点。注意这里的best_pose依然是在world下，只是对goal进行了偏移
    //但是如果利用worldToMap转化一下它的话，得到的一定是最适合的！
    //这一部分，好像就是强换版的validPointPotential？
    double resolution = costmap_->getResolution();
    geometry_msgs::PoseStamped p, best_pose;
    p = goal;
    bool found_legal = false;
    double best_sdist = DBL_MAX;
    p.pose.position.y = goal.pose.position.y - tolerance;
    while(p.pose.position.y <= goal.pose.position.y + tolerance)
    {
      p.pose.position.x = goal.pose.position.x - tolerance;
      while(p.pose.position.x <= goal.pose.position.x + tolerance)
      {
        double potential = getPointPotential(p.pose.position);
        double sdist = sq_distance(p, goal);
        if(potential < POT_HIGH && sdist < best_sdist){
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.pose.position.x += resolution;
      }
      p.pose.position.y += resolution;
    }

    //到这里说明goal可以转换到一个合适的map上
    //在这里对plan进行了赋值，与move_base联系
    if(found_legal){
      //这里是规划的入口，在里面又一次进行了规划？！？？
      //这个if之后，plan是完整的了
      if(getPlanFromPotential(best_pose, plan))
      {
        geometry_msgs::PoseStamped goal_copy = best_pose;
        goal_copy.header.stamp = ros::Time::now();
        plan.push_back(goal_copy);
      }
      else
      {
        ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
      }
    }

    //是否把potential作为点云进行发布，默认是false
    if (visualize_potential_)
    {
      // Publish the potentials as a PointCloud2
      sensor_msgs::PointCloud2 cloud;
      cloud.width = 0;
      cloud.height = 0;
      cloud.header.stamp = ros::Time::now();
      cloud.header.frame_id = global_frame_;
      sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
      cloud_mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                        "y", 1, sensor_msgs::PointField::FLOAT32,
                                        "z", 1, sensor_msgs::PointField::FLOAT32,
                                        "pot", 1, sensor_msgs::PointField::FLOAT32);
      cloud_mod.resize(planner_->ny * planner_->nx);
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");

      PotarrPoint pt;
      float *pp = planner_->potarr;
      double pot_x, pot_y;
      for (unsigned int i = 0; i < (unsigned int)planner_->ny*planner_->nx ; i++)
      {
        if (pp[i] < 10e7)
        {
          mapToWorld(i%planner_->nx, i/planner_->nx, pot_x, pot_y);
          iter_x[0] = pot_x;
          iter_x[1] = pot_y;
          iter_x[2] = pp[i]/pp[planner_->start[1]*planner_->nx + planner_->start[0]]*20;
          iter_x[3] = pp[i];
          ++iter_x;
        }
      }
      potarr_pub_.publish(cloud);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    /*
    for (std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();it!=plan.end();it++)
    {
      std::cout<<it->pose.position.x<<" "<<it->pose.position.y;
    }
    */
    return !plan.empty();
  }

  //发布路径给gazebo和rviz，用于可视化。
  //从这个函数可以看出global_planner绝对有问题
  //插：路径的四元素的orientation都是0,0,0，1。虽然不知道为啥。。
  void NavfnROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a)
  {
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //创建合适的Path gui_path 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    
    //设置gui_path的header
    if(path.empty()) 
    {
    	gui_path.header.frame_id = global_frame_;
      gui_path.header.stamp = ros::Time::now();
    } 
    else 
    { 
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    //对gui_path进行赋值，从这里就可以看出，那个global_planner的planner_core绝对有问题！
    for(unsigned int i=0; i < path.size(); i++)
    {
      gui_path.poses[i] = path[i];
    }

    //发布gui_path
    plan_pub_.publish(gui_path);
  }


  //这里把goal和plan输入进来
  //倒着进行规划
  //再倒一下生成plan
  //调用发布器，给gazebo和rviz
  bool NavfnROS::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    if(!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //清空之前的plan
    plan.clear();
    if(goal.header.frame_id != global_frame_)
    {
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    //goal的转换
    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }

    //这里相当于是把goal当作start给planner_进行规划了，所以导致后边是倒着来的
    //但是goal是啥啊。。
    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;
    planner_->setStart(map_goal);
    //这里应该是真正在进行plan？
    //因为之前plan.clear了
    planner_->calcPath(costmap_->getSizeInCellsX() * 4);
    //把plan中的清空提取出来，为发布作初始化
    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();
    ros::Time plan_time = ros::Time::now();
    //这是倒着来的，和前面对应。。map转到World然后调用publishPlan进行发布
    for(int i = len - 1; i >= 0; --i)
    {
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return !plan.empty();
  }
};
