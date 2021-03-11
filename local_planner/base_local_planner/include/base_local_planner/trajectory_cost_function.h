/*********************************************************************
 * Author: TKruse
 *********************************************************************/

#ifndef TRAJECTORYCOSTFUNCTION_H_
#define TRAJECTORYCOSTFUNCTION_H_

#include <base_local_planner/trajectory.h>

namespace base_local_planner 
{
//这个函数和nav_core差不多，也是单纯地定义了一个frame
class TrajectoryCostFunction 
{
public:

  /**
   *
   * General updating of context values if required.
   * Subclasses may overwrite. Return false in case there is any error.
   */
  virtual bool prepare() = 0;


  //返回trajectory的评分，虚函数，在这里没有被定义
  //做了个frame，输入一个路径traj，返回一个一个cost
  virtual double scoreTrajectory(Trajectory &traj) = 0;

  //获得scale
  double getScale() {
    return scale_;
  }
  //设置scale
  void setScale(double scale) {
    scale_ = scale;
  }

  virtual ~TrajectoryCostFunction() {}

protected:
  //就 设置了一下scale
  TrajectoryCostFunction(double scale = 1.0): scale_(scale) {}

private:
  double scale_;
};

}

#endif /* TRAJECTORYCOSTFUNCTION_H_ */
