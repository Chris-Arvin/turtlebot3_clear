/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/


/**********************************************************************
* brief         ------2020.5.12
* footprint本质上就是机器人的几个脚，把他们连线，机器人一定在这里多边形里面
  我觉得称为机器人轮廓会更好
* 注意，他这里并没有考虑orientation的问题
* 这个文件里面没有类，都是函数
**********************************************************************/






#ifndef COSTMAP_2D_FOOTPRINT_H
#define COSTMAP_2D_FOOTPRINT_H

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

namespace costmap_2d
{
//引用赋值，计算(0,0)到footprint的最大和最小距离
void calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint,
                                 double& min_dist, double& max_dist);

//本质上，point和point32是一样的
geometry_msgs::Point              toPoint(geometry_msgs::Point32 pt);
geometry_msgs::Point32            toPoint32(geometry_msgs::Point   pt);

//Point的vecotr和polygon之间的转换
geometry_msgs::Polygon            toPolygon(std::vector<geometry_msgs::Point> pts);
std::vector<geometry_msgs::Point> toPointVector(geometry_msgs::Polygon polygon);

//在原来的基础上，对所有的footprint进行x和y的平移，以及theta的旋转，得到新的x和y（姿态不重要的，这是机器人轮廓）
void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        std::vector<geometry_msgs::Point>& oriented_footprint);

//同上，只不过上面那个最后要打是point类型的vector，这里是polygonstamped类型的结果
void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        geometry_msgs::PolygonStamped & oriented_footprint);

//在原来的基础上，根据原来的符号进行扩大，增大一个padding
//实际上，就是把机器人往外扩大一个padding
void padFootprint(std::vector<geometry_msgs::Point>& footprint, double padding);

//给一个半径，以其为中心，360度分成16分，算出对应每个点的位置(x,y)
std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius);

//给一个类似于point的vector的string（例如[[1.1,1.2],[2.1,2.2]]），提取出内部的float，转化成point的vector的格式
//引用赋值
bool makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint);

//通过参数池构建，里面通过if判断类型，基本上调用了这里所有的函数
std::vector<geometry_msgs::Point> makeFootprintFromParams(ros::NodeHandle& nh);

//通过xml文件赋值，但我搞不太清楚里面的逻辑。。
std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                const std::string& full_param_name);
//把point的vector写成一个string，然后保存到nh的参数中
void writeFootprintToParam(ros::NodeHandle& nh, const std::vector<geometry_msgs::Point>& footprint);

}  // end namespace costmap_2d

#endif  // COSTMAP_2D_FOOTPRINT_H
