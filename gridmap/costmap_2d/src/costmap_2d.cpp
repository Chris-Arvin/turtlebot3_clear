/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/costmap_2d.h>
#include <cstdio>

using namespace std;

namespace costmap_2d
{
//这一块，初始化，给线程，参数赋值，一个地图大小确定，所有值都是default_value_的char*已经被定义完成了
Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                     double origin_x, double origin_y, unsigned char default_value) :
    size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x),
    origin_y_(origin_y), costmap_(NULL), default_value_(default_value)
{
  access_ = new mutex_t();
  initMaps(size_x_, size_y_);
  resetMaps();
}

//删地图，char* -> NULL
void Costmap2D::deleteMaps()
{
  // clean up data
  boost::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = NULL;
}

//delete之前的costmap_,创建一个1d的size_x * size_y的伪2d char*的数组
void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
  boost::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = new unsigned char[size_x * size_y];
}

//和初始化作用相同，重新定义地图所有的参数（先delete 后create），数值全是默认值
void Costmap2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                          double origin_x, double origin_y)
{
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  initMaps(size_x, size_y);
  resetMaps();
}

//通过地址赋值的方式，给costmap_赋值为default_value_，这个default在初始化的时候给
void Costmap2D::resetMaps()
{
  boost::unique_lock<mutex_t> lock(*access_);
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

//通过地址，一行一行的赋值，默认值
void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  boost::unique_lock<mutex_t> lock(*(access_));
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
    memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
}


//这里是把一个（可能）大一点的map，copy给了costmap_（先删再重新初始化）
//特殊调用了copyMapRegion，destination的大小是整个costmap_的大小
bool Costmap2D::copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                                  double win_size_y)
{
  // check for self windowing
  if (this == &map)
  {
    // ROS_ERROR("Cannot convert this costmap into a window of itself");
    return false;
  }

  // clean up old data
  deleteMaps();

  // compute the bounds of our new map
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y)
      || !map.worldToMap(win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y))
  {
    // ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
    return false;
  }

  size_x_ = upper_right_x - lower_left_x;
  size_y_ = upper_right_y - lower_left_y;
  resolution_ = map.resolution_;
  origin_x_ = win_origin_x;
  origin_y_ = win_origin_y;

  // initialize our various maps and reset markers for inflation
  initMaps(size_x_, size_y_);

  // copy the window of the static map and the costmap that we're taking
  copyMapRegion(map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_, size_y_);
  return true;
}

//重载了等号，用于初始化costmap_
//返回的是当前类的地址
Costmap2D& Costmap2D::operator=(const Costmap2D& map)
{
  // check for self assignement
  if (this == &map)
    return *this;

  // clean up old data
  deleteMaps();

  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  // initialize our various maps
  initMaps(size_x_, size_y_);

  // copy the cost map
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

//直接把自己地图的内容改成map的，注意这里是内容赋值，不是地址赋值
//this是指针，*this是内容
//这里是直接把map这个实例粘贴到初始化了，地址不同
Costmap2D::Costmap2D(const Costmap2D& map) :
    costmap_(NULL)
{
  access_ = new mutex_t();
  *this = map;
}

//全空的初始化
Costmap2D::Costmap2D() :
    size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
  access_ = new mutex_t();
}

Costmap2D::~Costmap2D()
{
  deleteMaps();
  delete access_;
}

//从world到map后，一个像素对应的正方形边长
//注意，int是向下取整
unsigned int Costmap2D::cellDistance(double world_dist)
{
  double cells_dist = max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}

//获得costmap_的接口
//从外面获得costmap_，可以直接用costmap2d这个类（直接括号初始化），
//也可以用下面这个函数，但是下面这个，需要自己区搞size啥的，有点麻烦
unsigned char* Costmap2D::getCharMap() const
{
  return costmap_;
}

//很奇怪，根据他的定义，mx和my就是以左上角为0,0对应的。。迷惑了
unsigned char Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  return costmap_[getIndex(mx, my)];
}
//设置
void Costmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  costmap_[getIndex(mx, my)] = cost;
}

//为啥这里有个0.5？？？
//他这里不对称啊？？？
void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

//有一步world点和origin的比较
bool Costmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_)
    return true;

  return false;
}

//这里是没有比较，硬返回
void Costmap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const
{
  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);
}

//将点限制在map内的worldtomap转化，如果超限，就设定为边界值
void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const
{
  if (wx < origin_x_)
  {
    mx = 0;
  }
  else if (wx >= resolution_ * size_x_ + origin_x_)
  {
    mx = size_x_ - 1;
  }
  else
  {
    mx = (int)((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_)
  {
    my = 0;
  }
  else if (wy >= resolution_ * size_y_ + origin_y_)
  {
    my = size_y_ - 1;
  }
  else
  {
    my = (int)((wy - origin_y_) / resolution_);
  }
}

//这里面利用local_map这个临时的中转，对costmap_进行了重写，用于update时被调用
//流程：把某区域的costmap的值给local_map，对costmap全部清0,对应位置local_map再给回costmap
//应该是默认了new_origin_x/y都是大于0的，不然会有错误（最后的copyMapRegion出错，start_x/y<0）
void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
  //和原来相同或变化极小（int向下取整）则不改变origin
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);
  if (cell_ox == 0 && cell_oy == 0)
    return;

  //注意这个new_grid_ox和输入的new_origin_x不一样，有一个int的影响（这样对吗？会产生偏差呀？）
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  //unsign int到int的转换
  int size_x = size_x_;
  int size_y = size_y_;
  //字面意思，求左下角和右上角。保证在0～size内
  //左下角坐标系
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = min(max(cell_ox, 0), size_x);
  lower_left_y = min(max(cell_oy, 0), size_y);
  upper_right_x = min(max(cell_ox + size_x, 0), size_x);
  upper_right_y = min(max(cell_oy + size_y, 0), size_y);

  //创建一个用来中转的地图local_map
  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;
  unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
  //把costmap以新的orgin为左下角，往右上扩展，赋值给local_map
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  //清空costmap_,把local_map粘贴到costmap_中
  resetMaps();
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  //删掉local_map
  delete[] local_map;
}

//通过给的polygon，把他内部的点也都填充成给的cost_value(去除unknown？)
bool Costmap2D::setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value)
{
  //把Point的点转变成map下自己定的map_polygon格式
  std::vector<MapLocation> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i)
  {
    MapLocation loc;
    if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y))
    {
      return false;
    }
    map_polygon.push_back(loc);
  }

  //调用convexFillCells进行填充
  std::vector<MapLocation> polygon_cells;
  convexFillCells(map_polygon, polygon_cells);
  for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  {
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    costmap_[index] = cost_value;
  }
  return true;
}

//这里自己调用自己是什么鬼。。
//他理论上应该是对所有的polygon，进行两两连接，但这个action啥的我是真的看不懂？
void Costmap2D::polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
  PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
  for (unsigned int i = 0; i < polygon.size() - 1; ++i)
  {
    raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
  }
  if (!polygon.empty())
  {
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
  }
}


//给一个polygin，先调用polygonOutlineCells进行边界填充，得到polygon_cells，然后再对每个x进行y方向的填充
void Costmap2D::convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells)
{
  //至少是一个三角形才可以填充
  if (polygon.size() < 3)
    return;

  //先填充边界（进行连线）
  polygonOutlineCells(polygon, polygon_cells);

  //按照x增大进行快排，卧槽还可以这么写
  MapLocation swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1)
  {
    if (polygon_cells[i].x > polygon_cells[i + 1].x)
    {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;
      if (i > 0)
        --i;
    }
    else
      ++i;
  }

  //初始化i和对应y的最大最小值
  i = 0;
  MapLocation min_pt;
  MapLocation max_pt;
  unsigned int min_x = polygon_cells[0].x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

  //对于同一个x，沿着y进行填充，push到polygon_cells中
  for (unsigned int x = min_x; x <= max_x; ++x)
  {
    if (i >= polygon_cells.size() - 1)
      break;

    if (polygon_cells[i].y < polygon_cells[i + 1].y)
    {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    }
    else
    {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x)
    {
      if (polygon_cells[i].y < min_pt.y)
        min_pt = polygon_cells[i];
      else if (polygon_cells[i].y > max_pt.y)
        max_pt = polygon_cells[i];
      ++i;
    }

    MapLocation pt;
    // loop though cells in the column
    for (unsigned int y = min_pt.y; y < max_pt.y; ++y)
    {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

//map的大小
unsigned int Costmap2D::getSizeInCellsX() const
{
  return size_x_;
}
unsigned int Costmap2D::getSizeInCellsY() const
{
  return size_y_;
}

//world的大小
double Costmap2D::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}
double Costmap2D::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

//origin & resolution
double Costmap2D::getOriginX() const
{
  return origin_x_;
}
double Costmap2D::getOriginY() const
{
  return origin_y_;
}
double Costmap2D::getResolution() const
{
  return resolution_;
}

//把地图写出保存
bool Costmap2D::saveMap(std::string file_name)
{
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  for (unsigned int iy = 0; iy < size_y_; iy++)
  {
    for (unsigned int ix = 0; ix < size_x_; ix++)
    {
      unsigned char cost = getCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

}  // namespace costmap_2d
