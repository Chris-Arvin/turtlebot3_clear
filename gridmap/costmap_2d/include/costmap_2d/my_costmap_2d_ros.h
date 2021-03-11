#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

#include<costmap_2d/layered_costmap.h>
#include<costmap_2d/layer.h>
#include<costmap_2d/costmap_2d_publisher.h>
#include<costmap_2d/Costmap2DConfig.h>
#include<costmap_2d/footprint.h>

#include<geometry_msgs/Polygon.h>
#include<geometry_msgs/PolygonStamped.h>
#include<geometry_msgs/PoseStamped.h>

#include<dynamic_reconfigure/server.h>
#include<plyginlib/class_loader.hpp>
#include<tf2/LinearMath/Transform.h>

class SuperValue: public XmlRpc::XmlRpcValue