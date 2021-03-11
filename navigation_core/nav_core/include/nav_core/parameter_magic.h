/*********************************************************************
* Author: David Lu
* 我怀疑这个函数的出现，就是因为在维护的时候，developer没有用原来体系的名字，而是自己用了一套，
  最后发现不行，就弄了一个前后名字的对应关系的函数。。
*********************************************************************/

#ifndef NAV_CORE_PARAMETER_MAGIC_H
#define NAV_CORE_PARAMETER_MAGIC_H

namespace nav_core
{

//模板函数，防止我们用了旧的名字，在这里返回了同样功能的新名字的值
template<class param_t>
param_t loadParameterWithDeprecation(const ros::NodeHandle& nh, const std::string current_name,
                                     const std::string old_name, const param_t& default_value)
{
  param_t value;
  if (nh.hasParam(current_name))
  {
    nh.getParam(current_name, value);
    return value;
  }
  if (nh.hasParam(old_name))
  {
    ROS_WARN("Parameter %s is deprecated. Please use the name %s instead.", old_name.c_str(), current_name.c_str());
    nh.getParam(old_name, value);
    return value;
  }
  return default_value;
}

//在dwa_planner_ros中的initialize中被调用，由于有了新的参数，所以不再需要用旧的参数了
//真实在被调用时，是因为本身我们用的是机器人能力限制的max和min参数，后来被人工赋值max和min后，不需要再用机器人最大限度来作为约束了
void warnRenamedParameter(const ros::NodeHandle& nh, const std::string current_name, const std::string old_name)
{
  if (nh.hasParam(old_name))
  {
    ROS_WARN("Parameter %s is deprecated (and will not load properly). Use %s instead.", old_name.c_str(), current_name.c_str());
  }
}

}  // namespace nav_core

#endif  // NAV_CORE_PARAMETER_MAGIC_H
