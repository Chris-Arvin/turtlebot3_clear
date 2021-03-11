/*********************************************************************
 * Author: TKruse
 *********************************************************************/

#ifndef FOOTPRINT_HELPER_H_
#define FOOTPRINT_HELPER_H_

#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>
#include <base_local_planner/Position2DInt.h>

namespace base_local_planner {

class FootprintHelper {
public:
  FootprintHelper();
  virtual ~FootprintHelper();


  //输入pos是当前位姿，footprint_spec是以机器人当前位置为中心的一系列footprint的顶点；
  //返回的是当前的机器人的外围的空心圈或者填充后的圈（根据fill）
  std::vector<base_local_planner::Position2DInt> getFootprintCells(
      Eigen::Vector3f pos,
      std::vector<geometry_msgs::Point> footprint_spec,
      const costmap_2d::Costmap2D&,
      bool fill);

  //给两个点，通过引用赋值pts，来填充一条线上的所有点。注意这里并不对pst先clear，只是一直后向push
  void getLineCells(int x0, int x1, int y0, int y1, std::vector<base_local_planner::Position2DInt>& pts);


  //对给定的footprint的填充。先沿x排序，然后x为主，y为辅助，一列一列的遍历来push footprint
  void getFillCells(std::vector<base_local_planner::Position2DInt>& footprint);
};

} /* namespace base_local_planner */
#endif /* FOOTPRINT_HELPER_H_ */
