/*********************************************************************
* Author: Eitan Marder-Eppstein
* Developer: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Reference：https://www.cnblogs.com/sakabatou/p/8297479.html
* Note:
  *   在move_base中与dwa_planner交互的流程：
          1. 把global_planner规划出来的路径通过setplan赋给dwa；
          以下会进行循环：
          2. 调用isGoalReached，判断目标是否到达了目标点。如果没到，则调用computeVelocityCommands来规划局部速度(引用赋值cmd_vel)；
          3. 在computeVelocityCommands中，先将全局路径映射到局部地图中，然后调用updatePlanAndLocalCosts更新对应的打分项；
          4. 利用:isPostionReached判断是否达到了目标position：
              如果到了，computeVelocityCommandsStopRotate，来计算对应的orientation旋转；
              如果没到，调用dwaComputeVelocityCommands，引用赋值，计算dwa局部速度（一个cmd_vel）。
          5. 调用findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored = 0)，查找最优的局部路径
              先调用prepare对每个打分器进行准备；
              调用TrajectorySampleGenerator中的产生器列表（目前这个列表只有一个SimpleTrajectoryGenerator）在(vx,vy,w)离散空间内进行轨迹生成和打分：
                  在打分时，调用scoreTrajectory对打分项进行求和（如果某个打分器的scale参数是0，则说明该打分项无效，跳过这个；如果有某个打分项是负的，则这个轨迹行不通，直接return）。
  *   在局部规划时，会创建一个以机器人自己为中心的局部地图，并且将global_planner的goal裁减到局部地图的边界处，作为新的goal'
  *   打分对象共有6个：
          base_local_planner::OscillationCostFunction oscillation_costs_（摆动打分）
          base_local_planner::ObstacleCostFunction obstacle_costs_（避障打分）
          base_local_planner::MapGridCostFunction path_costs_（路径跟随打分）
          base_local_planner::MapGridCostFunction goal_costs_（指向目标打分）
          base_local_planner::MapGridCostFunction goal_front_costs_（前向点指向目标打分）  当前点和目标点间的diff
          base_local_planner::MapGridCostFunction alignment_costs_（对齐打分）
          1. oscillation_costs_：判断有三个，机器人只能前行与生成后退速度的摆动；低速时机器人角速度较大的摆动（尽量不让机器人原地转圈圈）；二轮机器人不能有侧移；
          2. obstacle_costs_：因为local costmap是不进行膨胀的，所以这里需要对每个点求解footprint，然后检测障碍物碰撞（本质上和膨胀再检测点是一样的，TODO或者这里可以优化，因为膨胀检测的速度更快）；
        3和4的stop_on_failure为true，即如果轨迹中的某个点为障碍物或者未在路径计算地图中探索到，则表示查找该点的得分失败，返回负值
          3. path_costs_：这里主要是考虑local best trajectory每个点到global_plan的距离（global_paln的每个点的dist都是0，然后广搜赋值其他点）；
          4. goal_costs_：是path_costs_的另类版，只考虑两个global_plan的goal和local best trajectory的goal之间的dist。（只让global_plan的goal的dist=0，然后广搜其他，本质上这里只需要考虑local goal的dist就好，但是为了格式统一，浪费了一些速度）；
        5和6的stop_on_failure为false，不考虑障碍物碰撞和坐标系转换了，原理分别和3、4一样，只是他们把当前点(x,y)进行了一个偏移（5是x_shift，6是y_shift），TODO本质上考虑的是某个点的舒适空间？。
          5. goal_front_costs_：同3，用(x_shift,y)
          6. alignment_costs_：同4，用(x,y_shift)
*********************************************************************/

#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
//这个玩意挺有意思的。。
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

//把DWAPlannerROS注册成为BaseLocalPlanner的一个plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

  //利用config进行重新配置
  //这个level没用到呀。。
  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) 
  {
      //setup_默认为false，仅和reconfiguratino相关
      //如果已经re过了（即setup_为true），那么通过引用来把default_config_给config
      //否则config给default_config_
      if (setup_ && config.restore_defaults) 
      {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) 
      {
        default_config_ = config;
        setup_ = true;
      }

      //利用config更新参数
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  //初始化函数
  //setup_仅和reconfiguration相关，默认为false，表征是否re过;
  //initialized仅和isInitialized函数相关，表征是否被初始化过;
  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false) {

  }

  //这个initialize核心是实例化了DWAPlanner了的dp_
  //真正的初始化是通过planner_util_来实现的
  void DWAPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) 
  {
    if (! isInitialized()) 
    {

      ros::NodeHandle private_nh("~/" + name);
      //两个publisher 用于可视化
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      //赋值地图（指针），引用赋值当前pose
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      //持续更新，注意这里是最基础的那个costmap2D*
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      //？我不太清楚boost这套体系，但是这应该是真正的planner
      //下面的dp_是dwa_planner中DWAPlanner类的初始化，是真正作规划的东西
      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());
      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      //？
      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      //初始化成功
      initialized_ = true;

      //这里实在进行了一个说明，由于配置了max和min的一些东西，所以不需要再使用机器人最大限度的配置了
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      //reconfiguration的server和对应的callback，并进行了设置
      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  //这里是对base_local_planner的一个框架行的初始化，本质上就是调用dp_的setplan来把global的plan放进去
  bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    //这里是干啥。。
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  //判断是否达到了goal
  //核心是调用了latchedStopRotateController_，借助planner_util_,odom_helper_和curret_pose_来判断是否到了goal
  bool DWAPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  //俩发布器，publishPlan是在goal_function中定义的
  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }
  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS::~DWAPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }


  //引用赋值cmd_vel
  //最重要的是调用了dp_->findBestPath来求解local path和robot_vel（当前的tiwst）
  //注意cmd_vel通过pub与底盘交互
  //local plan的求解只是为了可视化
  //输入的global_pose是当前位姿，cmd_vel是会被引用赋值的
  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) 
  {
    if(! isInitialized())
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //用posestamped类型生成robot_vel，伪twist，x->vx, y->vy, yaw->w
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    //速度的frameID(LOCAL MAP)
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
    
    //调用findBestPath，在多条dp_规划出来的path中找到最好的traj，且对robot_vel进行了赋值
    //参数：当前pose，当前速度，待求解的输出速度（引用赋值）
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    //pass along drive commands
    //引用赋值，把drive_cmds的形式转换成twist
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) 
    {
      ROS_DEBUG_NAMED("dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    //遍历path，来对local_plan进行赋值
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
    }

    //publish information to the visualizer
    publishLocalPlan(local_plan);
    return true;
  }



  //引用赋值cmd_vel
  //实际上是调用了dwacomputeVelocityCommands
  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) 
  {
    //不能获得机器人pose
    if ( ! costmap_ros_->getRobotPose(current_pose_)) 
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    //调用getLocalPlan来初始化transformed_plan
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) 
    {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //对transformed_plan进行检测，如果是空的，直接false退出
    if(transformed_plan.empty()) 
    {
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    //调用dp_的update函数
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());
    
    //是否到达了position（旋转相关）
    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) 
    {
      //如果到达了目标position，发布空的plan给global和local的publisher
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      //调用latchedStopRotateController_
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
    }
    //如果没有到达目标position 
    else 
    {
      //调用dwaComputeVelociityCommands，如果成功就发布transformed_plan，失败则发布空的
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) 
      {
        publishGlobalPlan(transformed_plan);
      } 
      else 
      {
        ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }


};
