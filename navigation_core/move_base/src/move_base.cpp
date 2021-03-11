r/*********************************************************************
* Author: Eitan Marder-Eppstein
* Developer：Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update date：2020.7.14
*********************************************************************/

/*********************************************************************
* Updated log: 
* 1. 添加了中文注释。 ------2020.4.15
* 2. 对代码进行了简化。
     1) 去除了make_plan_srv_, clear_costmaps_srv_及他们对应的callback。（即不对外部提供规划的接口了）;
     2) 去除了reconfigure及其对应的subscriber。      ------2020.5.7
     3) 进行了代码和注释的规范化。       ------2020.7.14
*********************************************************************/


/*
逻辑梳理：
整个文件是以MoveBaseActionServe（service的优化版）为入口，以as_对其进行了实例化;
每当as_接收到一个goal时：
先调用executeCb[一切的一切的入口，就是靠他里面的while来进行输出和控制的]，来进行一系列初始化操作;
在executeCb内部，调用了executeCycle，根据state_的不同来进行plan，control，recovery。

thread是一个很重要的函数，在里面进行了多个配置和makePlan（真正在plan，state_的plan也是通过开启该线程来实现的）。
这个文件，很巧秒的运用了开关。state_ 和 runPlanner_ 这两个key起了很大的作用，
runPlanner_会在goal合法后变成true，只有它为true，才可以开启规划线程。

本质上，这个文件的publish都是为了gazebo和rviz而发布的，不对规划起到任何作用。
*/


/*
通信：
Pulisher
current_goal_pub_: geometry_msgs::PoseStamped, "current_goal", 0。与gazebo交互。由executeCb发布。
vel_pub_: geometry_msgs::Twist, "cmd_vel" , 1 。与base和gazebo交互。由局部规划器（tc_）发出。
action_goal_pub_: move_base_msgs::MoveBaseAcionGoal, "goal", 1。有goalCB发出。这玩意好像就是个对PoseStamped的包装，没多大区别。先接受了goal，后来做了封装。

Subsriber 
goal_sub_: geometry_msgs::PoseStamped, “goal”，1 ，goalCB。这个对应上面的action_goal_pub_，接受PoseStamped的goal，转化为MoveBaseActionGoal类型，再发出来。
好像，同一个topic可以接收不同类型的数据。当尝试从topic获取内容时，subscriber会检测数据类型？

ServiceServer
make_plan_srv_: "make_plan", planService。这里面的req包含start和goal，resp是规划出的一系列点。调用这个server，会让系统根据已经配置好的初始化和给出的req，重新规划一条道路路径，并返回。
（但他并不会对系统内部已经作出的costmap和plan等作出任何改变，相当于只是分出来了一块空白，把配置加进来，给了外界一个接口）。
（make_plan_srv这个server是可以删除的，对主程序没有影响）
clear_costmaps_src_: "clear_costmaps", clearCostmapsService。这里面没有利用req，resp也是空的，知识对地同进行了重置（全为0）。
*/




#include <move_base/move_base.h>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {
  /*
  * 初始化整个movebase
  */
  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),
    as_(NULL),  //用null初始化movebaseactionserver
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),    //两个costmap2DROS的初始化
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),    //对三个插件进行初始化
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) 
    {
      //actionlib是一个优化版的service，可以查看进度和取消
      as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
      //这里的“～”是创建了该node自己的namespace，在这里，他的参数的namespace是move_base/~ ???
      //关于ros::init和ros::NodeHandle：
      //init是在初始化一个node，而NodeHandle是在帮这个node和ROS sysytem交互！获取一些参数值！
      //其实只写一个NodeHandle就行，但是为了可读，一般会写好几个，每个只负责自己的部分
      ros::NodeHandle private_nh("~");
      ros::NodeHandle nh;

      recovery_trigger_ = PLANNING_R;

      //接收上层的parameters: 上层parameter的名字，在这里的名字，缺省时用的值
      std::string global_planner, local_planner;
      //这里默认的global planner是navfn的dijkstra;local是Tra。。
      private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
      private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
      private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
      private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
      //这里planner_frequency默认是0,即如果不起动规划launch，即使启动了move_base，也不会自己进行规划
      private_nh.param("planner_frequency", planner_frequency_, 0.0);
      private_nh.param("controller_frequency", controller_frequency_, 20.0);
      private_nh.param("planner_patience", planner_patience_, 5.0);
      private_nh.param("controller_patience", controller_patience_, 15.0);
      private_nh.param("max_planning_retries", max_planning_retries_, -1);  //默认-1，表征不启动
      //振荡相关，和recovery有关系
      private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
      private_nh.param("oscillation_distance", oscillation_distance_, 0.5);
      //make_plan service的参数，clear是清空的意思（区域内全部赋值为0）
      private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
      private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

      //创建三个和plan有关的空vector，上次规划的path、最新规划的path、控制规划（局部最优）的path
      planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
      latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
      controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

      //planner的线程
      planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

      //和底盘间的交互 command the base
      vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);  //发送给下位机的速度，整个上位机最大的意义、唯一的输出
      current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );  //发布当前位姿，用于rviz显示，没有逻辑意义
      ros::NodeHandle action_nh("move_base");
      action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
            
      //下面这里的subscriber是为了接受用户通过terminor或rviz发的goal，即2Ddestination
      //这里通过goalCB处理后得到实际进行规划的目标
      ros::NodeHandle simple_nh("move_base_simple");
      goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

      //内切圆，外界圆，重构，保守重设半径;关闭，重构，恢复是否开启
      private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
      private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
      private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
      private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);
      private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
      private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
      private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

      //初始化全局规划器：
      //注意在这里 planner_ 是global planner的；
      //这里的map是2d的一个增强包the ros wrapper for the planner's costmap
      planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
      planner_costmap_ros_->pause();
      //初始化全局规划，bgp_loader是一个实例（nav_core的插件BaseGlobalPlanner的实例）；
      //planner_是实例化插件的实例，执行的是global_planner的方法
      try
      {
        planner_ = bgp_loader_.createInstance(global_planner);
        ROS_INFO("Created global_planner %s",global_planner.c_str());
        planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
      } 
      catch (const pluginlib::PluginlibException& ex) 
      {
        //fatal:致命的
        //因没有实例化插件而导致加载失败，错误退出exit(1)
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
        exit(1);
      }

      //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
      controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
      controller_costmap_ros_->pause();

      //同上，初始化局部规划器：
      //这里是 tc_
      try 
      {
        tc_ = blp_loader_.createInstance(local_planner);
        ROS_INFO("Created local_planner %s", local_planner.c_str());
        tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
      } 
      catch (const pluginlib::PluginlibException& ex) 
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
        exit(1);
      }

      //激活，更新costmap的地图
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();

      //如果是shutdown状态，不进行地图更新（默认是shutdown的）
      if(shutdown_costmaps_)
      {
        ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
        planner_costmap_ros_->stop();
        controller_costmap_ros_->stop();
      }
      //查看是否加载了恢复方法，如果没有，加载默认的
      if(!loadRecoveryBehaviors(private_nh))
      {
        loadDefaultRecoveryBehaviors();
      }

      //设置初始状态为正在规划中
      state_ = PLANNING;
      //初始初始恢复行为为0（PLANNING_R）
      recovery_index_ = 0;

      //截止到目前，所有的setup已经完成了，接下来开启action server，等待命令
      as_->start();

    }


  //在这里实际上是针对goal这个topic的，先接受一个PoseStamped形式的goal，再在同名topic中发布一个MoveBaseActionGoal的goal
  //注意，虽然这里的topic的名字都是goal，但是因为msg的类型不同，因该并不是同一个topic
  //相当于是从A接受goal，更改格式，然后发到了B
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
  {
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    //获取当前时间，注意在ROS系统中有一套模拟时间;一般会用duration设置最大延时
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;
    action_goal_pub_.publish(action_goal);
  }

  //针对全局和局部两个地图，分别在以机器人为中心的矩形区域内进行清空（赋值为0）
  //两次操作相同，只是针对的地图不同
  void MoveBase::clearCostmapWindows(double size_x, double size_y)
  {
    /*clear the planner's costmap*/
    //获取机器人当前位置x和y
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    //Point这种msg是平面的点坐标，只有xy
    std::vector<geometry_msgs::Point> clear_poly;
    geometry_msgs::Point pt;
    //创建矩形的clear窗口
    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);
    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);
    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);
    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);
    //在planner_costmap_ros_中，将clear_poly这个矩形区域内所有的点，设置为0（FREE_SPACE）
    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    /*clear the controller's costmap*/
    getRobotPose(global_pose, controller_costmap_ros_);
    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;
    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);
    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);
    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);
    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);
    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  //delete所有能delete的
  MoveBase::~MoveBase()
  {
    recovery_behaviors_.clear();
    if(as_ != NULL)
      delete as_;
    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;
    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;
    planner_thread_->interrupt();
    planner_thread_->join();
    delete planner_thread_;
    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;
    planner_.reset();
    tc_.reset();
  }

  //！！！！！！！！
  //这个makeplan是对上面那个planservice的简化版
  //理论上它才应该是最重要的呀。。！！
  //在里面进行了路径规划
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));
    plan.clear();
    //是否已经初始化地图
    if(planner_costmap_ros_ == NULL) 
    {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }
    //是否能够获得机器人位姿
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) 
    {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    //进行规划
    const geometry_msgs::PoseStamped& start = global_pose;
    if(!planner_->makePlan(start, goal, plan) || plan.empty())
    {
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }
    return true;
  }

  //机器人停止运动。Twist六个参数，其他三个恒为0
  void MoveBase::publishZeroVelocity()
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  //这里是在检测当前的位姿变换是否合法（绕z轴且有意义）
  //geometry_msgs::Quaternion是四元数消息类型
  //里面有一些函数我没弄太清楚。。
  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q)
  {
    //检测quaternion是否有错
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    {
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }
    //复制构造
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    //接近0，则失败，为啥？length2好像是平方和？
    if(tf_q.length2() < 1e-6)
    {
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //接下来是检测，旋转轴是不是近似z轴
    tf_q.normalize();
    tf2::Vector3 up(0, 0, 1);
    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));
    if(fabs(dot - 1) > 1e-3)
    {
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }
    return true;
  }

  //这里主要是把系统给的goal从原始的frame_id转到global下（planner_costmap）的描述下
  //但是有一个问题是，即使转化失败，也只会返回原始的goal，无法分辨是否转换成功？
  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
  {
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    goal_pose.header.stamp = ros::Time();
    try
    {
      //transform会根据第一个参数的信息（尤其是frameid）和第三个参数的信息，进行转换
      //来得到第二个参数（引用赋值）
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex)
    {
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  //唤醒线程。本质上是通过设定频率，定时休眠，定时唤醒来实现的
  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    planner_cond_.notify_one();
  }

  //在整个程序中，最开始进行了一些绑定，然后通过make_Plan来进行Planning
  //这里面是通过开关来控制是否执行后续线程的。最重要的是runPlanner这个key，每次更改他都需要先锁lock。
  //这个程序里面，没有完整的退出机制啊，只能等程序中断，故意的？？应该是。。
  void MoveBase::planThread()
  {
    //ROS_DEBUG_NAMED是把后面的信息输出到前面名字的log中
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    //while(n.ok)的意思是，只要这个节点启动，就一直循环执行
    while(n.ok())
    {
      //双开关并联控制，只要wait_for_wake为true或者runPlanner_为false中有一个成立，则挂起线程
      while(wait_for_wake || !runPlanner_)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();
      //!!!!
      //开始plan,在这里调用了makePlan
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");
      planner_plan_->clear();
      //在这里调用makeplan，搞出了路径
      //通过指针/引用方式，将规划出的plan放到了planner_plan中
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if(gotPlan)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        //swap地址，planner_plan_由于储存上一次的path，latest_plan储存最新的
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        //flag，标志new plan完成
        new_global_plan_ = true;
        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");
        //下面是为了确保，仅在没有达到goal的时候，启动controller
        //如果当前是runPlanner的状态，那么通过上述操作，就已经可以转变为CONTROLLING状态了
        if(runPlanner_)
          state_ = CONTROLLING;
        //这个planner_frequency默认是0的，即默认把runPlaner变为false
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }

      //state_初始化是PLANNING的
      //这里有一个隐含条件是else if，即如果gotPlan，则一定不会执行下面的东西
      //如果没有gotPlan，说明还处于PLANNING状态，这是有问题的（plan失败）
      //这里通过planning_retries_++，来使得循环继续，直到达到某个阈值，退出规划
      else if(state_==PLANNING)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        lock.lock();
        //每到这里一次，尝试规划次数就加一，直到到达阈值，state_变成CLEARING（规划失败，清空退出）
        planning_retries_++;
        if(runPlanner_ && (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_)))
        {
          state_ = CLEARING;
          runPlanner_ = false;
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }
      //为下一次iteration作准备（先暂停）
      lock.lock();
      if(planner_frequency_ > 0)
      {
        //进行本次循环所用的时间
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        //如果时间超限，让这个while休眠（关闭一个开关）
        if (sleep_time > ros::Duration(0.0))
        {
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  //！！！
  //进行了goal的转换和一些合法性检验;
  //输入是目标位姿
  //发布了当前的goal: current goal
  //调用了executeCycle，是规划的入口
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    //四元数不合法，错误退出
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
    //转换goal坐标系到global frame
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
    publishZeroVelocity();

    //开启planner线程
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //发布当前的goal
    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;
    ros::Rate r(controller_frequency_);

    //这个shutdown_costmaps参数，全程就没有改变过。。一共通过true调用了三次：stop->start->stop
    //也就是说，如果最开始输入的是true，则进行上述;如果是false，则全程开启（在最初的初始化中start了）
    //这个地方的shutdown+start应该不是写错了，而是做一个规划开始前的地图初始化
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //更改为当前时间戳
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {
      //是否更改controller_frequency。有默认的值，默认的c_freq_change是false;如果reconfig的话，更改controller_frequency
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }
      //preempt是抢占的意思，这一行说明可以抢占线程？
      if(as_->isPreemptRequested())
      {
        //可以获得新goal
        if(as_->isNewGoalAvailable())
        {
          //判断new_goal是否四元数合法
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
          {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          //将goal转换到globalframe（其实在这里本身也是globalframe的，这里只是为了确保）
          goal = goalToGlobalFrame(new_goal.target_pose);

          recovery_index_ = 0;
          state_ = PLANNING;
           //锁线程，改goal，唤起规划线程，解锁
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
          //发布当前goal到topic：current goal
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);
          //确定时间戳没问题
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else 
        {
          //重置机器人状态，告诉机器人可以抢占线程了
          resetState();
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();
          return;
        }
      }

      //检测global frame是否发生了了改变
      //注意，这个goal是已经做过一次goalToGlobalFrame的，这里如果不相等，只能是改变了globalframeID
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
      {
        //重新变换坐标系
        goal = goalToGlobalFrame(goal);
        //更改状态
        recovery_index_ = 0;
        state_ = PLANNING;
        //锁线程，确定goal和plan状态，唤醒线程，解锁线程
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();
        //发布current goal
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);
        //更改时间戳
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //！！！！！！
      //在这里调用了executeCycle！！找到辣！！！
      ros::WallTime start = ros::WallTime::now();
      bool done = executeCycle(goal, global_plan);
      if(done)
        return;
      //规划后的报错
      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());
      r.sleep();
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }//while结束的那个括号
    //锁线程，是否规划过了，唤醒线程，解锁线程。这里是按照规范写，还是改变runPlanner必须要锁线程？？
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();
    //不能成立
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  //hypot,计算三角形的直角边（平方和开根号）
  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }


  //其实这里是真正在干事儿，别的地方要不是配合他，要不是做预处理，要不是入口或退出机制。
  //输入的goal是目标位姿，global_plan是会被引用赋值的全局规划路径点（实际上global_plan也没有被用到。。）
  //如果到达了目标点，则返回true
  //这个函数之前已经进行好了全局规划，在这里通过state来控制调用哪部分：PLANNING，CONTROLLING，CLEARING
  //PLANNING就开启了一下规划线程？？
  //CONTROLLING是进行局部规划
  //CLEARING是进行恢复，有自己的3个恢复依据

  //这个cycle很有趣的是，每次都只执行一次动作（或失败，planning_retries++），然后就返回，等待调用后，再执行一次动作。
  //（即使是recovery也是执行一步，而不是把所有的recovery一口气执行完。万一只recovery一下就可以planning和controlling成功呢？）
  //这也是为什么它可以实现实时规划的原因！

  //在这里，stete_和recovery_trigger_的key作用体现的很好
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan)
  {
    //初始化
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::PoseStamped global_pose;
    //获得global_pose
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;//引用
    //发布feedback（MBF形式的current_position）
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);
    //?????
    //这里的oscillation_pose_并没有被初始化啊。。默认为0？？
    //这里的原注释为：check to see if we've moved far enough to reset our oscillation timeout
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }
    //costmap是不是最新的（误差允许范围内）
    //通过发布的cmd与gazebo/base交互
    if(!controller_costmap_ros_->isCurrent())
    {
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }
    //new_global_plan_默认false，在thread中被开启，即如果规划过一次路径了，他就一定是true的
    //这里就是：如果已经规划出路径了，则将最新的全局规划赋值给局部规划器作为参考路径，准备要进行控制规划了
    if(new_global_plan_)
    {
      new_global_plan_ = false;
      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      //swap两个plan
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;   
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      //如果局部规划器setplan失败，说名有问题了，要退出了
      if(!tc_->setPlan(*controller_plan_))
      {

        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();
        //结合之前和下面，可以发现，在修改runPlanner时，一定要锁死线程
        lock.lock();
        runPlanner_ = false;
        lock.unlock();
        //这里为啥返回的是true？。。
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //根据state_做相应的处理
    switch(state_)
    {
      case PLANNING:
        {
          //开启规划线程
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");
        //检测是否达到目标点
        if(tc_->isGoalReached())
        {
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();
          //关闭规划线程
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }
        //检测振荡情况
        //停止运动，装遍state为clearing，确定恢复状态
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }
        //这里之前都是CONTROLLING的预处理？不是的，case加不加{}都行。。
        {
          boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
          //！！！这里是真正计算速度的入口
          //进行局部规划，算出速度并发布到cmd_vel
          if(tc_->computeVelocityCommands(cmd_vel))
          {
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
          }
          //局部规划失败的处理机制
          else 
          {
            ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
            //这个ros：Duration实际上是把括号里double类的15转变为ros模拟系统中的15秒（sec）
            ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
            //规划时间超过阈值，失败
            if(ros::Time::now() > attempt_end)
            {
              publishZeroVelocity();
              state_ = CLEARING;
              recovery_trigger_ = CONTROLLING_R;
            }
            //在规划时间内，但是局部规划失败，说明是全局规划有问题，所以把state重新设置为PLANNING
            else
            {
              //停止运动
              last_valid_plan_ = ros::Time::now();
              planning_retries_ = 0;
              state_ = PLANNING;
              publishZeroVelocity();

              //锁线程，开启规划
              boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
              runPlanner_ = true;
              planner_cond_.notify_one();
              lock.unlock();
            }
          }
        }

        break;

      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
        {
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
          recovery_behaviors_[recovery_index_]->runBehavior();
          last_oscillation_reset_ = ros::Time::now();
          //注意在cycle里面，每次执行玩一次动作就把state_转变为PLANNING，因为是实时在规划的。
          //即使是在CLEARING中，也是执行一次CLEARING就回去试一试规划
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          //尝试（失败）次数
          planning_retries_ = 0;
          state_ = PLANNING;
          recovery_index_++;
        }
        //如果能到这里，说明plan和control都失败了，而且也执行完了所有的recovery。当前goal是一定行不通的
        //要说明是为什么行不通了
        else
        {
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");
          if(recovery_trigger_ == CONTROLLING_R)
          {
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R)
          {
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R)
          {
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      //state_既不是PLANNING又不是CONTROLLING又不是CLEARING，一定是程序出问题了，直接退出，去debug叭骚年！
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }
    //如果真的进行到这一步。。那是见了鬼了。。
    return false;
  }


  //加载了recovery函数
  //注意recovery是一系列的动作，所以是一个数组性质的recovery（参考default）
  //顺序执行，即可恢复
  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
  {
    //What the f??这大概是一种数组
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list))
    {
      //应该是数组形式了
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        //遍历
        for(int i = 0; i < behavior_list.size(); ++i)
        {
          //辣个list里面的每个都应该是一个struct
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            //struct必须包含的内容
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type"))
            {
              //这一层循环是为了确定behavior_list中没有重复的
              for(int j = i + 1; j < behavior_list.size(); j++)
              {
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type"))
                  {
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            //这里是在list里面的元素struct的结构不是name和type时的处理机制
            else
            {
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          //不是struct时的处理机制
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //如果能运行到这里，说明获得了behavior这个参数，而且没有问题，结构啥的都正确
        for(int i = 0; i < behavior_list.size(); ++i)
        {
          try
          {
            //这里主要是在检测有没有名字不完整的，只有type对，别的不对的。用标准class替换？
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"]))
            {
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i)
              {
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i]))
                {
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }
            //调用插件的createInstance进行初始化
            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex)
          {
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else
      {
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else
    {
      return false;
    }
    return true;
  }


  //除了外部提供来的recovery接口，自己有一套已经写好的recovery代码
  //分别是
  //用于clear costmap的 clear_cosmap_recovery/ClearCostmapRecovery
  //用于旋转恢复的rotate_recovery/RotateRecovery
  //重复执行了两次，一次是conservative的，一次是aggressive的？？
  void MoveBase::loadDefaultRecoveryBehaviors()
  {
    recovery_behaviors_.clear();
    try
    {
      //获取参数
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);
      //首先，加载默认插件，清空地图 保护的重置？
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //其次，加载旋转插件
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_)
      {
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //其次，再次清空地图，强制重置？？
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //最后，再一次旋转重置？？
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }


  //创建新线程，让机器人停止
  void MoveBase::resetState()
  {
    //创建线程并解锁
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();
    //更改规划状态，恢复方式，恢复状态，机器人停止
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();
    //是否关闭更新地图
    if(shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }


  //引用赋值，生成global_pose，输入为全局costmap
  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    //软换时间超限，直接报错
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
