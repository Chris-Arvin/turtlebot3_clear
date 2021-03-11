/*********************************************************************
 * Author: TKruse
 * Arvin: 依然是一个frame。。
 *********************************************************************/

#ifndef TRAJECTORY_SAMPLE_GENERATOR_H_
#define TRAJECTORY_SAMPLE_GENERATOR_H_

#include <base_local_planner/trajectory.h>

namespace base_local_planner {

/**
 * @class TrajectorySampleGenerator
 * @brief Provides an interface for navigation trajectory generators
 */
class TrajectorySampleGenerator {
public:

  /**
   * Whether this generator can create more trajectories
   */
  virtual bool hasMoreTrajectories() = 0;

  /**
   * Whether this generator can create more trajectories
   */
  virtual bool nextTrajectory(Trajectory &traj) = 0;

  /**
   * @brief  Virtual destructor for the interface
   */
  virtual ~TrajectorySampleGenerator() {}

protected:
  TrajectorySampleGenerator() {}

};

} // end namespace

#endif /* TRAJECTORY_SAMPLE_GENERATOR_H_ */
