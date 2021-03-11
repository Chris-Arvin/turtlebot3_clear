/*********************************************************************
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>

namespace base_local_planner {
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples) {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }

  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) {
    double traj_cost = 0;
    int gen_id = 0;
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) 
      {
        continue;
      }
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) 
      {
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      if (cost != 0) 
      {
        cost *= score_function_p->getScale(); //各个评分器的评分，只要不是小于0，那就都是1
      }
      traj_cost += cost;
      if (best_traj_cost > 0) 
      {
        // since we keep adding positives, once we are worse than the best, we will stay worse
        if (traj_cost > best_traj_cost) {
          break;
        }
      }
      gen_id ++;
    }


    return traj_cost;
  }

  //输入的这俩都是需要被赋值的。。
  //traj是best traj，每个点的保存格式是(x,y,yaw)
  //all_explored是所有可行的traj，保存每个轨迹的完整信息
  //核心函数是调用gen_->nextTrajectory和scoreTrajectory
  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) 
  {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    //遍历打分器，判断是否prepare完毕
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) 
    {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }

    //遍历路径生成器（这里其实就一个）
    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) 
    {
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      //对每个路径生成器，调用nextTrajectory来生成一条轨迹，然后进行打分，选择分数最高的那个作为best_traj
      while (gen_->hasMoreTrajectories()) 
      {
        //成功生成一条轨迹
        gen_success = gen_->nextTrajectory(loop_traj);
        if (gen_success == false) 
        {
          // TODO use this for debugging
          continue;
        }
        //进行score
        loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
        //这条traj被检测过了
        if (all_explored != NULL) 
        {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }
        //有效traj计数增加
        if (loop_traj_cost >= 0) 
        {
          count_valid++;
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;
          }
        }
        //迭代次数++
        count++;
        //知道遍历gen_
        if (max_samples_ > 0 && count >= max_samples_) {
          break;
        }        
      }

      //把best_traj引用赋值给traj，push进去(x,y,yaw)
      if (best_traj_cost >= 0) 
      {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);
        }
      }
      ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      if (best_traj_cost >= 0) {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }

  
}// namespace
