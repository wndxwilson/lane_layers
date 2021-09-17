#ifndef LANE_LAYER_H_
#define LANE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <list>

namespace lane_layer_namespace
{

class LaneLayer : public costmap_2d::Layer
{
public:
  LaneLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  ros::Subscriber sub;

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  void laneCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  double mark_x_, mark_y_;
  std::string global_frame_;
  std::vector<geometry_msgs::PointStamped> point_array_transform;
  std::vector<geometry_msgs::Pose> point_array;
  std_msgs::Header header;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif