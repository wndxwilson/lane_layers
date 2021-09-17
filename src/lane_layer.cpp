#include<lane_layers/lane_layer.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <geometry_msgs/PointStamped.h>

PLUGINLIB_EXPORT_CLASS(lane_layer_namespace::LaneLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace lane_layer_namespace
{

LaneLayer::LaneLayer() {}

void LaneLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &LaneLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  sub = nh.subscribe("/lane_points",1,&LaneLayer::laneCallback,this);
  global_frame_ = layered_costmap_->getGlobalFrameID();
}


void LaneLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void LaneLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  
  point_array_transform.clear();

  for (unsigned int i = 0; i < point_array.size(); i++){
      geometry_msgs::PointStamped pt, opt;
      
      try{
          opt.point= point_array[i].position;
          opt.header = header;
        //   tf_->transform(pt,opt,global_frame_);
          point_array_transform.push_back(opt);
      }
      catch (tf2::LookupException& ex)
        {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        continue;
        }
  }

  mark_x_ = robot_x + 2*cos(robot_yaw);
  mark_y_ = robot_y + 2*sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void LaneLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  unsigned int mx;
  unsigned int my;

  for (unsigned int i = 0; i < point_array_transform.size(); i++){
    if(master_grid.worldToMap(point_array_transform[i].point.x,point_array_transform[i].point.y, mx, my)){
        ROS_WARN("%d , %d ",mx,my);
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }
}

void LaneLayer::laneCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ROS_WARN("GOT message");
    header = msg->header;
    point_array = msg->poses;
}
} // end namespace