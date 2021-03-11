/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 * Developer: Arvin
 * E-mail: 1711506@mail.nankai.edu.cn
 *********************************************************************/


 /*
Coding notes        ------2020.5.11
* 定义函数的时候用const，挺好用的; 模板也挺好玩的
* inline的一些相关知识 https://www.runoob.com/w3cnote/cpp-inline-usage.html
* 这里面的costmap都是char*的
* 在.h里面已经定义了很多函数了,这里面的都是inline
* 他的mapToWorld 和 worldToMap不是逆的呀？？？？
* cpp的286行，有问题啊，会有int带来的偏差的，故意的？
* ！！！特别需要注意的是，costmap在建立的时候，索引值就是以地图左下角为原点这样建立的。也就是说costmap中的（1,3）实际上就是map中以左下角origin为原点，（1,3）的那个值
  我们并不需要考虑y颠倒的关系，因为costmap的建立本身就是颠倒的。不需要把数组画出来再怎样怎样，直接考虑map中的坐标和costmap中的坐标是相同的就可以了！！
  即，我们想获取真实map上的(x，y)，只需要区get costmap中的(x,y)就可以了！确实有一个视觉上的上下颠倒，但是逻辑上是完全没问题的！！
  即，虽然图形确实是颠倒的，但是index值完全对应的！
*/


#ifndef COSTMAP_2D_COSTMAP_2D_H_
#define COSTMAP_2D_COSTMAP_2D_H_

#include <vector>
#include <queue>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>

namespace costmap_2d
{

  //自定义的结构体
  struct MapLocation
  {
    unsigned int x;
    unsigned int y;
  };


class Costmap2D
{
  friend class CostmapTester;  // Need this for gtest to work correctly
public:
  //四个初始化，注意default_value是0
  Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
            double origin_x, double origin_y, unsigned char default_value = 0);
  Costmap2D(const Costmap2D& map);
  Costmap2D();
  Costmap2D& operator=(const Costmap2D& map);
  virtual ~Costmap2D();

  //这里是把一个（可能）大一点的map，copy给了costmap_（先删再重新初始化）
  //特殊调用了copyMapRegion
  bool copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
                         double win_size_y);
  //get & set cost
  unsigned char getCost(unsigned int mx, unsigned int my) const;
  void setCost(unsigned int mx, unsigned int my, unsigned char cost);

  //有个问题，他们并不是互逆的，有一个0.5的偏移
  void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
  
  void worldToMapNoBounds(double wx, double wy, int& mx, int& my) const;
  void worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const;

  //index和cell之间的转换
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return my * size_x_ + mx;
  }
  inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
  {
    my = index / size_x_;
    mx = index - (my * size_x_);
  }


  unsigned char* getCharMap() const;

  //获得x和y的长度
  unsigned int getSizeInCellsX() const;
  unsigned int getSizeInCellsY() const;

  double getSizeInMetersX() const;
  double getSizeInMetersY() const;

  double getOriginX() const;
  double getOriginY() const;
  double getResolution() const;

  //default value
  void setDefaultValue(unsigned char c)
  {
    default_value_ = c;
  }
  unsigned char getDefaultValue()
  {
    return default_value_;
  }

  //结合着用，给几个点，把内部填充
  bool setConvexPolygonCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value);
  void polygonOutlineCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);
  void convexFillCells(const std::vector<MapLocation>& polygon, std::vector<MapLocation>& polygon_cells);
  //重新定义相关
  virtual void updateOrigin(double new_origin_x, double new_origin_y);
  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                 double origin_y);
  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);
  unsigned int cellDistance(double world_dist);

  bool saveMap(std::string file_name);
  // Provide a typedef to ease future code maintenance
  typedef boost::recursive_mutex mutex_t;
  mutex_t* getMutex()
  {
    return access_;
  }

protected:
  //实际用到的data_type就是char*，这里是地图部分copy
  /*
  把source_map的以sm_lower_left为起点，region的区域，赋值到dest_mao的以dm_lower_left为起点，region的区域
  不要想太多，他这个costmap在建立的时候，就是索引值和实际map的坐标值对应，
  只考虑index对应，不要考虑图形的颠倒。
  */
  template<typename data_type>
    void copyMapRegion(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y)
    {
      data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
      data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);
       //以一行为单位，每次赋值一行，移动一行
      for (unsigned int i = 0; i < region_size_y; ++i)
      {
        memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
        sm_index += sm_size_x;
        dm_index += dm_size_x;
      }
    }


  virtual void deleteMaps();
  virtual void resetMaps();
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

  //给出一条线，两端点是(x0,y0)和(x1,y1)对这条线间所有的点进行一个操作at。里面大多是在做初始化
  //在内部调用的bresenham2D是真的在对每个点进行操作
  //注意这里的sign是自己定义的，用于提取正负号
  template<class ActionType>
    inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                             unsigned int max_length = UINT_MAX)
    {
      //线的偏移值
      int dx = x1 - x0;
      int dy = y1 - y0;
      unsigned int abs_dx = abs(dx);
      unsigned int abs_dy = abs(dy);
      //提取dx和dy的正负号
      int offset_dx = sign(dx);
      int offset_dy = sign(dy) * size_x_;
      //从原点到x0 y0的偏移值
      unsigned int offset = y0 * size_x_ + x0;
      //变换尺度？有一个映射关系？
      double dist = hypot(dx, dy);
      double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
      //选择dx和dy中大的一个，/2作为error，输入给bresenham2D
      if (abs_dx >= abs_dy)
      {
        int error_y = abs_dx / 2;
        bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        return;
      }
      int error_x = abs_dy / 2;
      bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
    }

private:
  //这个inline是不是长了点。。可能在执行时，无法通过inline实现
  //这个函数，是以dx和dy大的那一个为基础，判断当前一步是该沿着x方向动一下还是y方向动一下
  template<class ActionType>
    inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                            int offset_b, unsigned int offset, unsigned int max_length)
    {
      unsigned int end = std::min(max_length, abs_da);
      for (unsigned int i = 0; i < end; ++i)
      {
        at(offset);
        offset += offset_a;
        error_b += abs_db;
        if ((unsigned int)error_b >= abs_da)
        {
          offset += offset_b;
          error_b -= abs_da;
        }
      }
      at(offset);
    }

  inline int sign(int x)
  {
    return x > 0 ? 1.0 : -1.0;
  }

  mutex_t* access_;


protected:
  unsigned int size_x_;
  unsigned int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  unsigned char* costmap_;
  unsigned char default_value_;


  //类中类
  //单纯地定义了一张地图，然后重载了()，对里面的index，赋值为value_
  class MarkCell
  {
  public:
    MarkCell(unsigned char* costmap, unsigned char value) :
        costmap_(costmap), value_(value)
    {
    }
    inline void operator()(unsigned int offset)
    {
      costmap_[offset] = value_;
    }
  private:
    unsigned char* costmap_;
    unsigned char value_;
  };

  //这里的char_map_好像根本没用到啊。。
  //这个函数是给了一个地图，给了一个vector。输入一个index，转换成对应的x和y坐标，push到vector中
  class PolygonOutlineCells
  {
  public:
    PolygonOutlineCells(const Costmap2D& costmap, const unsigned char* char_map, std::vector<MapLocation>& cells) :
        costmap_(costmap), char_map_(char_map), cells_(cells)
    {
    }

    // just push the relevant cells back onto the list
    inline void operator()(unsigned int offset)
    {
      MapLocation loc;
      costmap_.indexToCells(offset, loc.x, loc.y);
      cells_.push_back(loc);
    }

  private:
    const Costmap2D& costmap_;
    const unsigned char* char_map_;
    std::vector<MapLocation>& cells_;
  };
};
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_H
