//
// Created by chengdaqian on 17-11-19.
//

#ifndef UAV_COSTMAP_OBSTACLE_LAYER_UAV_OBSTACLE_LAYER_H
#define UAV_COSTMAP_OBSTACLE_LAYER_UAV_OBSTACLE_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObstaclePluginConfig.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/obstacle_layer.h>

namespace uav_obstacle_layer
{
class UAVObstacleLayer : public costmap_2d::ObstacleLayer
{
public:
    UAVObstacleLayer()
    {
        costmap_ = NULL;
    }

    virtual void onInitialize();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x, double* max_y);

    std::vector<bool> inflated_visited_;

protected:

    // below which we see as ground, and above as obstacle.
    // note that this param is different from min_obstacle_height_, which is for data-filtering purpose
    double obstacle_height_threshold_;

};
}


#endif //UAV_COSTMAP_OBSTACLE_LAYER_UAV_OBSTACLE_LAYER_H

