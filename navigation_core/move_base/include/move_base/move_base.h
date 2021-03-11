/*********************************************************************
* Author: Eitan Marder-Eppstein
* Developer：Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update date：2020.7.14
*********************************************************************/

/*********************************************************************
* 重要开关
  1. bool runPlanner_ 是否开启规划线程，在planThread()种，如果他是false，则会一直循环等待（即不会调用全局规划器，不会让机器人开始导航，锁死）
  2. state_；planning、controlling、clearing，三个执行状态
*********************************************************************/

/*********************************************************************
* 重要线程
  1. 主线程：move_base + executeCb + executeCycle
  2. 唯一的子线程：planner线程：路径规划线程，以runplanner和state_为开关进行控制
  3. costmap_ros的更新，他并没有自己的线程，但是可以抽象一下
*********************************************************************/

/*********************************************************************
* 代码特点
  1. 用引用来赋值；大多数函数都是bool的返回值，只表征是否规划成功
  2. 错误退出机制写的很完善
*********************************************************************/

/*********************************************************************
* 逻辑梳理
  1. 在初始化函数中，初始化三个规划器+全局、局部costma。调用as_->start开启server，实时接收目标位姿。
  2. 当接收到目标点位姿时，调用executeCb，首先更新一下两个costmap，然后设置状态state_=planning（确定下个动作是进行规划），
      通过while(n.ok())，死循环，调用executeCycle()，执行全局规划和控制规划，有问题就执行恢复规划。
  3. 在executeCycle中，首先执行控制或恢复的初始化，然后通过switch决定是进行planning、controlling还是recovery
*********************************************************************/

#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>   //路径的vector
#include <string>   //主要用来保存handle和topic的name
#include <ros/ros.h>    //ros系统的主要文件

#include <actionlib/server/simple_action_server.h>    //这是一个优化版的server，接收目标位姿，调用规划线程
#include <move_base_msgs/MoveBaseAction.h>    //用于获得rviz中选取的目标位姿
#include <geometry_msgs/PoseStamped.h>        //用于传递位姿信息
#include <nav_msgs/GetPlan.h>

#include <nav_core/base_local_planner.h>    //局部规划/控制规划器
#include <nav_core/base_global_planner.h>   //全局规划器
#include <nav_core/recovery_behavior.h>     //恢复规划器
#include <pluginlib/class_loader.hpp>       //插件文件，用于加载上述三个规划器。实际上插件和调用头文件的效果是一样的，只是插件形式使程序更规整，方便更改程序（直接在launch中改，不许要在一堆文件中繁琐地#include。。。了）

#include <costmap_2d/costmap_2d_ros.h>    //costmap地图，由slam+map_sever构建的图片解析而来
#include <costmap_2d/costmap_2d.h>


#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>   //在执行过程中，重新配置（更改插件或参数等）。这一套move_base的特点之一就在于可以通过server的形式去实时更改配置
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  /**
  * 重命名actionserver
  */
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;
  /**
  * 机器人接下来需要执行的指令
  */
  enum MoveBaseState {
    PLANNING, //全局路径规划
    CONTROLLING,  //局部路径规划，输出速度
    CLEARING  //恢复规划，清空局部地图
  };

  /**
  * 机器人恢复类型
  */
  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
  * 机器人导航的类。通过movebaseactionserver的形式被调用，接收目标位姿，反复进行规划
  */
  class MoveBase {
    public:
      /**
       * 初始化所有参数
       * 初始化全局和局部costmap：new实例化；pause地图更新；加载规划器；start地图更新；根据"shutdown_costmaps"（默认false，即不停止）来判断是否stop更新
       * 加载全局、局部、恢复规划器
       * 开启as_这个movebaseactionserver，等待发送机器人目标位姿的client来call
       */
      MoveBase(tf2_ros::Buffer& tf);

      /**
       * 析构函数
       */
      virtual ~MoveBase();

      /**
       * 输入为global frame下的目标位姿goal，会被引用赋值的全局规划路径点global_plan（实际上并没有被后续使用，可删）
       * 根据偏差判断是否需要执行recovery；
         根据new_global_plan_是否为true，判断是否要锁规划线程+给controller_plan_赋值+开启规划线程+控制规划器setplan
         switch（state_），进入核心控制部分：
            Planning：唤醒一次规划线程进行规划
            Controlling： 
                        如果到达了goal，则runPlanner_=false，抛弃这个goal；
                        如果发生了震荡，则速度置0，state_=Clearing, recovery_trigger_=OSCILLATION_R;
                        调用控制规划期tc_->computeVelocityCommands(), 计算cmd_vel，
                            如果计算失败，如果控制规划的时间较长，尝试进行地图清空，state_=Clearning, recovery_trigger_ = controlling_R;
                                        如果控制时间步长，就是单纯地失败了，可能是因为规划的路径不好，需要重新规划全局路径，state_=planning，并且唤醒一下规划线程进行一次规划
            Clearing：如果恢复是可执行的且恢复执行数组还没执行完：通过index+1的形式执行一次恢复行为，state_=planning
                      不然的话，runplanner_=false，规划线程等待；抛弃这个goal，重置机器人，返回true。
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:

      /**
       * 输入目标位姿goal，会被引用赋值规划好的路径plan
       * 返回是否规划成功
       * 在里面，实际上是调用全局规划plugin的makeplan进行规划
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * 初始化recovery_behaviors_时push_back了三次：clear_costmap + rotate_recovery + clear_costmap
       * 注意，在每次进入executeCb中的循环的时候recovery_index_都会被置为0；resetstate()也会
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * 输入是清空窗口在x和y方向的长度
       * 针对全局和局部costmap，分别进行清空（全部设置为0，即free）
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * cmd_vel全0，发布topic给下位机
       */
      void publishZeroVelocity();

      /**
       * 重置机器人状态，runplanner_=false,state_=planning,recovery_trigger=Planning_R,
       * 发布全0的速度
       * 停止更新costmap
       */
      void resetState();

      /**
      * 把接收到的posestamped形式的goal转换成movebaseactiongoal重新发布
      * 对整个程序的逻辑没有影响，不知道意义是什么，可能是为了rviz显示
      */
      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      /**
      * 规划线程，核心是调用了make_plan
      * 开启了一个死循环while(n.ok())
        如果wait_for_wake==true 和 runPlanner==False中有一个成立，则循环等待
        清空全局规划路径的vector planner_plan_
        调用全局规划器的makePlan，引用赋值planner_plan_
          如果规划成功，先锁住线程，将state_设置为CONTROLLING（此时PLANNING已经完成了），将runPlanner_设置为false，然后解锁线程。【因为在更改线程内的参数或变量时一定需要先锁定线程，防止错乱】
          如果规划失败，则讲尝试规划次数++，state_改为CLEARING，runplanner_改为false（即进行恢复规划，更新局部地图，直到恢复完成，runplanner才会被变成true，再次进行全局路径规划）
        锁定线程，更改休眠时间的参数
      */
      void planThread();

      /**
      * 输入是目标位姿
      * 设置规划线程为开启状态，更新了一次全局和局部地图（todo但是规划器的地图已经被初始化过了呀。。）
        while(n.ok())持续执行下面的操作
        {
          如果as_这个server线程可否被抢占执行：
            如果as_的goal已经设置了，state_更改为planning；
            【存在return跳出】如果没有设置，则调用resetState，机器人停止，state_=planning，runPlanner=false（这样，planthread就会一直卡着），停止更新costmap,直接return。
          如果goal不是在全局坐标系下的：
            将goal转换到全局坐标系，state_=planning，runplanner_=true（基本同as_可被强占且goal已经设置好了的情况）
          【核心】调用executeCycle来进行循环，如果到达目标点了就返回true，成功退出；否则抛弃这个goal，报目标点设置错误。
        }
      */
      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);
      
      /**
      * 检查四元素是否合法，主要是确定四元素平方和=1，且z轴近似=0
      */
      bool isQuaternionValid(const geometry_msgs::Quaternion& q);
      
      /**
      * 利用tf_，引用赋值global_pose，返回的是输入的costmap的id下的位姿
      */
      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      /**
      * 输入为两个点的位姿
      * 返回为两个点的2D距离
      */
      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      /**
      * 利用tf_将goal转换到全局坐标系
      */
      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       * 唤醒规划器进行一次规划（本质上是把plan那个线程从wait中拿出来执行一次）
       * 里面就是一个planner_cond_.notify_one();
       * 实际上根本没有调用这个函数，都是直接写的planner_cond_.notify_one()....
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf2_ros::Buffer& tf_;   //通过transformlistener来监听一堆坐标系及他们的转换方程。保存的是如何进行转换

      MoveBaseActionServer* as_;    //用于接收目标位姿的server，通过它被call，开启导航规划
      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;    //局部规划器
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_; //全局规划的地图，局部规划的地图

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;    //全局规划器
      std::string robot_base_frame_, global_frame_;   //机器人坐标系，全局坐标系

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;    //恢复规划器
      unsigned int recovery_index_;   //恢复规划的index（一共三个，被枚举了）

      geometry_msgs::PoseStamped global_pose_;    //全局位姿(实际上并没有被用到)
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_; //全局规划频率，控制规划频率，内切圆半径，外接圆半径
      double planner_patience_, controller_patience_; //todo 全局规划容忍最大规划时间，局部规划容忍最大规划时间
      int32_t max_planning_retries_;    //尝试规划的最大次数
      uint32_t planning_retries_;       //尝试规划的次数
      double conservative_reset_dist_, clearing_radius_;  //这两个参数没有用到

      ros::Publisher current_goal_pub_;   //发布当前位姿，用于rviz显示，没有逻辑意义
      ros::Publisher vel_pub_;    //发送给下位机的速度，整个上位机最大的意义、唯一的输出
      ros::Publisher action_goal_pub_;    
      ros::Subscriber goal_sub_;  //用于接受来自rviz的goal的位姿

      bool shutdown_costmaps_;  //是否关闭实时更新costmap
      bool clearing_rotation_allowed_, recovery_behavior_enabled_;  //是否允许进行旋转清空恢复规划（边原地旋转边更新costmap），是否允许进行恢复规划
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;   //这两个参数没有用到
      double oscillation_timeout_, oscillation_distance_;  //todo 好像是偏移量

      MoveBaseState state_;     //规划状态类型
      RecoveryTrigger recovery_trigger_;  //恢复规划器的类型

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_; //记录时间，判断超时
      geometry_msgs::PoseStamped oscillation_pose_;   //todo这里好像有一点问题，他在第一次被调用前，并没有被赋值过。。

      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;  //插件加载器 全局规划
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;   //插件加载器 局部规划
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;  //插件加载器 恢复规划

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_; //全局规划器 上一次规划的路径
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;  //全局规划器 最新规划出的路径，这里有一个细节，我们每次进局部规划时，是把上一次的全局路径planner_plan_作为参考路径的，有一点延迟的感觉
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;  //局部控制器 规划的路径

      //set up the planner's thread
      bool runPlanner_;   //是否开启规划线程（是否进行规划）
      boost::recursive_mutex planner_mutex_;    //规划线程
      boost::condition_variable_any planner_cond_;  //条件变量，notify_one是唤醒一个wait在条件变量上的子线程
      geometry_msgs::PoseStamped planner_goal_;   //全局规划的目标位姿
      boost::thread* planner_thread_;   //规划线程


      boost::recursive_mutex configuration_mutex_;    //重新设置参数或规划器的线程

      bool setup_;    //setup_默认为false，但是没有被用到过。。
      bool p_freq_change_, c_freq_change_;    //是否更改规划频率、控制频率
      bool new_global_plan_;    //是否获得了新的全局规划路径
  };
};
#endif

