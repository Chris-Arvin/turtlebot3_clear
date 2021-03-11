/*********************************************************************
 * Author: TKruse
 * Note: 特别注意一下vector3f这个东西，他是(x,y,w)
 * 核心函数是generateTrajectory()，在里面调用了computevelocity和position。
    里面的机理是：给定一个target_vel，这是我们期望的机器人能保持平稳运行的速度；
                里程计读取当前速度，把当前速度和target_vel作对比，判断机器人该加速还是减速；（一个traj开始时，被pub给底盘的速度其实是dt后的速度，这也是我们最核心要求的东西）
                在dt内会按照匀速走到达position；【computeposition计算】
                dt之后产生一个v=v+at/v-at的跳变；【computevelocity计算】（这个v和sample_target_vel做比较，sample_target_vel是我们期望机器人能保持平稳运行的速度）
                如此循环。

                实际上，我们的速度是进行跳变的。（即在当前状态下，我pub给底盘的速度是dt后应该有的）
                但是因为dt很小，而且我们是实时算法，所以就无所谓了。所有这种控制算法是一定有误差的。
 * 注意，我们要的其实就是当前时间+dt后的那个cmd_vel（被pub给下位机），而所谓的best traj，只是为了找到这个cmd_vel而做出的多步拟合
 *********************************************************************/

#ifndef SIMPLE_TRAJECTORY_GENERATOR_H_
#define SIMPLE_TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <Eigen/Core>

namespace base_local_planner {

/**
 * generates trajectories based on equi-distant discretisation of the degrees of freedom.
 * This is supposed to be a simple and robust implementation of the TrajectorySampleGenerator
 * interface, more efficient implementations are thinkable.
 *
 * This can be used for both dwa and trajectory rollout approaches.
 * As an example, assuming these values:
 * sim_time = 1s, sim_period=200ms, dt = 200ms,
 * vsamples_x=5,
 * acc_limit_x = 1m/s^2, vel_x=0 (robot at rest, values just for easy calculations)
 * dwa_planner will sample max-x-velocities from 0m/s to 0.2m/s.
 * trajectory rollout approach will sample max-x-velocities 0m/s up to 1m/s
 * trajectory rollout approach does so respecting the acceleration limit, so it gradually increases velocity
 */
class SimpleTrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator {
public:

  SimpleTrajectoryGenerator() 
  {
    limits_ = NULL;
  }

  ~SimpleTrajectoryGenerator() {}

  //这个initialise调用了下面那个，并且在sample_params_后添加了additional_samples的内容
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      std::vector<Eigen::Vector3f> additional_samples,
      bool discretize_by_time = false);

  //设置sample_params_：在xyz三个方向，从velocity的min到max，离散出vsamples[0/1/2]个点
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      bool discretize_by_time = false);

  //设置参数
  void setParameters(double sim_time,
      double sim_granularity,
      double angular_sim_granularity,
      bool use_dwa = false,
      double sim_period = 0.0);

  //可以继续生成路径
  bool hasMoreTrajectories();

  //在sample_params_换下一个参数进行generateTrajectory，看能否成功生成路径。
  bool nextTrajectory(Trajectory &traj);

  //输入的当前位置为pos，输入的速度为vel(vx,vy,w)，行走时间dt
  static Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel, double dt);

  //给一个sample_target_vel，是我们在整个traj中期望机器人可以保持的速度
  //如果它小于当前速度vel，则机器人减速，取max(sample_target_vel,v-at)；否则机器人加速，取min(sam..,v+at)，作为下一时刻的真实速度。
  //参数：采样速度，当前速度，机器人动力约束，单位时间dt
  static Eigen::Vector3f computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
      const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt);

  
  //参数：当前pose，当前速度（里程计获得），采样的速度，待引用赋值的轨迹
  //在内部调用了computeNewVelocities，一步步仿真来生成traj
  //通过里程计读取当前速度，生成轨迹后，被pub的速度其实是一个dt后的速度
  bool generateTrajectory(
        Eigen::Vector3f pos,
        Eigen::Vector3f vel,
        Eigen::Vector3f sample_target_vel,
        base_local_planner::Trajectory& traj);

protected:

  unsigned int next_sample_index_;
  std::vector<Eigen::Vector3f> sample_params_;  //xyz三个方向上采样拉伸后的速度vector
  base_local_planner::LocalPlannerLimits* limits_;  //和机器人动力约束有关
  Eigen::Vector3f pos_; //当前的xy三个方向的线速度和yaw的角速度 (x,y,w)
  Eigen::Vector3f vel_; 

  //速度是否变化
  bool continued_acceleration_; //持续加速 （= ! use_dwa_）
  bool discretize_by_time_; //是否被时间离散化

  double sim_time_;
  double sim_granularity_;  //障碍物探测的长度单元
  double angular_sim_granularity_;  //障碍物探测的角度单元
  bool use_dwa_;    //是否使用dwa
  double sim_period_; //点之间的距离
};

} /* namespace base_local_planner */
#endif /* SIMPLE_TRAJECTORY_GENERATOR_H_ */
