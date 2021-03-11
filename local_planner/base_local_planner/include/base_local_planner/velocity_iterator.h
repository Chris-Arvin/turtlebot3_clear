/*********************************************************************
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef DWA_LOCAL_PLANNER_VELOCITY_ITERATOR_H_
#define DWA_LOCAL_PLANNER_VELOCITY_ITERATOR_H_
#include <algorithm>
#include <cmath>
#include <vector>

namespace base_local_planner {

  /**
   * We use the class to get even sized samples between min and max, inluding zero if it is not included (and range goes from negative to positive
   */
  class VelocityIterator {
    public:
      //对速度进行离散化，从min到max离散num_samples个
      VelocityIterator(double min, double max, int num_samples):
        current_index(0)
      {
        if (min == max) 
        {
          samples_.push_back(min);
        } 
        else 
        {
          num_samples = std::max(2, num_samples);

          // e.g. for 4 samples, split distance in 3 even parts
          double step_size = (max - min) / double(std::max(1, (num_samples - 1)));

          // we make sure to avoid rounding errors around min and max.
          double current;
          double next = min;
          for (int j = 0; j < num_samples - 1; ++j) 
          {
            current = next;
            next += step_size;
            samples_.push_back(current);
            // if 0 is among samples, this is never true. Else it inserts a 0 between the positive and negative samples
            if ((current < 0) && (next > 0)) 
            {
              samples_.push_back(0.0);
            }
          }
          samples_.push_back(max);
        }
      }

      //获得当前速度
      double getVelocity(){
        return samples_.at(current_index);
      }

      //速度点加一
      VelocityIterator& operator++(int){
        current_index++;
        return *this;
      }
      
      //重置（速度从头开始输出，而不是让vector清零）
      void reset(){
        current_index = 0;
      }

      //是否完成
      bool isFinished(){
        return current_index >= samples_.size();
      }

    private:
      std::vector<double> samples_;
      unsigned int current_index;
  };
};
#endif
