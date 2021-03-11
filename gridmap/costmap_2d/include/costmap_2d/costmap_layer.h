/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_LAYER_H_
#define COSTMAP_2D_COSTMAP_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

namespace costmap_2d
{

class CostmapLayer : public Layer, public Costmap2D
{
public:
  CostmapLayer() : has_extra_bounds_(false),
    extra_min_x_(1e6), extra_max_x_(-1e6),
    extra_min_y_(1e6), extra_max_y_(-1e6) {}

  bool isDiscretized()
  {
    return true;
  }

  //默认的是将Costmap_定义为master，进行了初始化
  virtual void matchSize();

  //除了这两个点组成的矩形范围外，其他全部清空(NO_INFORMATION,默认255)
  virtual void clearArea(int start_x, int start_y, int end_x, int end_y);

  //m0和m1一个是左上角一个是右下角，他对原有的(private变量定义)bounding box进行了扩展（实际上是在定义box)）
  //并酱has_extra_bounds_改为了true
  void addExtraBounds(double mx0, double my0, double mx1, double my1);

protected:
  //下面这四个个函数都涉及到了一个enabled_的开关，如果false，则不执行
  //都是在将costmap_（当前layer的地图）的值赋给master_grid，只不过实现的方式/函数不太一样
  //全部通过引用赋值

  //用这个函数里面的costmap_区更新后四个参数的box对应部分的master_grid值
  void updateWithTrueOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  //这个和上面一样，只是多了一步检测，如果sotmap_的值是NO_INFORMATION(255)，则停止检测
  void updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  //同之前，取两者中大的进行赋值
  void updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  //同之前，差别在于，这里是直接在原来的基础上加
  //如果msater_grid的值是NO_INFORMATION，则直接正常赋值;若不是，则在原基础上加和赋值(赋值后与252取min)
  void updateWithAddition(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  //原始的矩形框架(后四个参数)
  //给定(x,y)，将框架扩展到包含(x,y)点
  void touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y);

  //首先需要调用addExtraBounds来使4个private变量合法，然后调用useExtraBound通过指针获得值
  //他们两个函数间通过一个has_extra_bounds_来进行通信，防止未add就use
  //其实有点多此一举。。直接定义完就用多好。。
  void useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y);
  bool has_extra_bounds_;

private:
  //这四个参数定义了一个extra bounding box，该box默认不错在，如果通过addExtrabounds进行了赋值，
  //并调用了useExtrabounds，通过指针获地了bound，他们就会发生作用
  double extra_min_x_, extra_max_x_, extra_min_y_, extra_max_y_;
};

}  // namespace costmap_2d
#endif  // COSTMAP_2D_COSTMAP_LAYER_H_
