/*********************************************************************
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <boost/bind.hpp>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/cost_values.h>

namespace costmap_2d
{

char* Costmap2DPublisher::cost_translation_table_ = NULL;

Costmap2DPublisher::Costmap2DPublisher(ros::NodeHandle * ros_node, Costmap2D* costmap, std::string global_frame,
                                       std::string topic_name, bool always_send_full_costmap) :
    node(ros_node), costmap_(costmap), global_frame_(global_frame), active_(false),
    always_send_full_costmap_(always_send_full_costmap)
{
  costmap_pub_ = ros_node->advertise<nav_msgs::OccupancyGrid>(topic_name, 1,
                                                    boost::bind(&Costmap2DPublisher::onNewSubscription, this, _1));
  costmap_update_pub_ = ros_node->advertise<map_msgs::OccupancyGridUpdate>(topic_name + "_updates", 1);

  if (cost_translation_table_ == NULL)
  {
    cost_translation_table_ = new char[256];

    // special values:
    cost_translation_table_[0] = 0;  // NO obstacle
    cost_translation_table_[253] = 99;  // INSCRIBED obstacle 内切障碍物
    cost_translation_table_[254] = 100;  // LETHAL obstacle 致命的障碍物
    cost_translation_table_[255] = -1;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 253; i++)
    {
      cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
    }
  }

  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

Costmap2DPublisher::~Costmap2DPublisher()
{
}

void Costmap2DPublisher::onNewSubscription(const ros::SingleSubscriberPublisher& pub)
{
  prepareGrid();
  pub.publish(grid_);
}

// prepare grid_ message for publication.
void Costmap2DPublisher::prepareGrid()
{
  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  double resolution = costmap_->getResolution();

  grid_.header.frame_id = global_frame_;
  grid_.header.stamp = ros::Time::now();
  grid_.info.resolution = resolution;

  grid_.info.width = costmap_->getSizeInCellsX();
  grid_.info.height = costmap_->getSizeInCellsY();
  
  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  grid_.info.origin.position.x = wx - resolution / 2;
  grid_.info.origin.position.y = wy - resolution / 2;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  saved_origin_x_ = costmap_->getOriginX();
  saved_origin_y_ = costmap_->getOriginY();

  grid_.data.resize(grid_.info.width * grid_.info.height);

  unsigned char* data = costmap_->getCharMap();
  for (unsigned int i = 0; i < grid_.data.size(); i++)
  {
    grid_.data[i] = cost_translation_table_[ data[ i ]];
  }
}

void Costmap2DPublisher::publishCostmap()
{
  //检测其subscriber的数量，还能这么做卧槽
  if (costmap_pub_.getNumSubscribers() == 0)
  {
    return;
  }

  float resolution = costmap_->getResolution();

  if (always_send_full_costmap_ || grid_.info.resolution != resolution ||
      grid_.info.width != costmap_->getSizeInCellsX() ||
      grid_.info.height != costmap_->getSizeInCellsY() ||
      saved_origin_x_ != costmap_->getOriginX() ||
      saved_origin_y_ != costmap_->getOriginY())
  {
    prepareGrid();
    costmap_pub_.publish(grid_);
  }
  else if (x0_ < xn_)
  {
    boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    // Publish Just an Update
    map_msgs::OccupancyGridUpdate update;
    update.header.stamp = ros::Time::now();
    update.header.frame_id = global_frame_;
    update.x = x0_;
    update.y = y0_;
    update.width = xn_ - x0_;
    update.height = yn_ - y0_;
    update.data.resize(update.width * update.height);

    unsigned int i = 0;
    for (unsigned int y = y0_; y < yn_; y++)
    {
      for (unsigned int x = x0_; x < xn_; x++)
      {
        unsigned char cost = costmap_->getCost(x, y);
        update.data[i++] = cost_translation_table_[ cost ];
      }
    }
    costmap_update_pub_.publish(update);
  }

  xn_ = yn_ = 0;
  x0_ = costmap_->getSizeInCellsX();
  y0_ = costmap_->getSizeInCellsY();
}

}  // end namespace costmap_2d
