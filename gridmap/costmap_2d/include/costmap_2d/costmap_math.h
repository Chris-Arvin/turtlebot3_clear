/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 * Developer: Arvin
 * E-mail: 1711506@mail.nankai.edu.cn
 *********************************************************************/

/*********************************************************************
* 这里是costmap会用到的部分数学公式
* 其中distanceToLine好像有点bug（其实不是常规的求点到线的距离）
**********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_MATH_H_
#define COSTMAP_2D_COSTMAP_MATH_H_

#include <math.h>
#include <algorithm>
#include <vector>
#include <geometry_msgs/Point.h>

//返回正负号
inline double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

//返回正负0
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

//Euclidean distance
inline double distance(double x0, double y0, double x1, double y1)
{
  return hypot(x1 - x0, y1 - y0);
}

//求p点到0和1这两点连线的距离。如果点p在01组成的线内，则求距离，否则求0和1两点中距离P最近的点的距离
double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

//给一个vector和一个点(testx,testy)，判断是否相交
bool intersects(std::vector<geometry_msgs::Point>& polygon, float testx, float testy);

//判断两个数组是否相交
bool intersects(std::vector<geometry_msgs::Point>& polygon1, std::vector<geometry_msgs::Point>& polygon2);

#endif  // COSTMAP_2D_COSTMAP_MATH_H_
