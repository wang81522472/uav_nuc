//
// Created by chengdaqian on 18-4-10.
//

#ifndef SIMPLE_MAP_2D_SIMPLE_MAP_2D_ROS_H
#define SIMPLE_MAP_2D_SIMPLE_MAP_2D_ROS_H

#include <simple_map_2d/simple_map_2d.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>

namespace simple_map_2d
{
    class SimpleMap2DROS
    {
    public:
        SimpleMap2DROS(std::string name, std::string global_frame_name);
        ~SimpleMap2DROS();

        SimpleMap2D *map_;

        void pubMap() const;

    private:
        void loadParams();

        void setBoundaryObstacle();

        void ugv_odom_cb(const nav_msgs::Odometry &ugv_odom);

        void uav_odom_cb(const nav_msgs::Odometry &uav_odom);

        /**
         * @brief ugv_sensor (laser) call_back, transform data into map_frame and update the map accordingly
         * @param ugv_scan: ugv_sensor reading.
         */
        void ugv_sensor_cb(const sensor_msgs::LaserScan   &ugv_scan);

        /**
         * @brief uav_sensor (point_cloud) call_back, transform data into map_frame and update the map accordingly
         * @param uav_points: ugv_sensor reading.
         */
        void uav_sensor_cb(const sensor_msgs::PointCloud2 &uav_points);

        void pubLaser(const Eigen::ArrayXXd& ranges, double center_x, double center_y);


        void pubCloud(const Eigen::MatrixX4f& points);

    private:
        ros::NodeHandle nh_, private_nh_;
        ros::Subscriber ugv_odom_sub_, uav_odom_sub_;
        ros::Subscriber ugv_sensor_sub_, uav_sensor_sub_;
        ros::Publisher  grid_map_pub_, grid_map_inflated_pub_, laser_cloud_pub_, uav_cloud_pub_;

        // data containers
        geometry_msgs::Pose ugv_pose_, uav_pose_;
        double ugv_tf_offset_x_, ugv_tf_offset_y_, uav_tf_offset_x_, uav_tf_offset_y_;
        Eigen::Matrix4f tfZedToBody_;

        // system params
        bool is_sim_, is_has_uav_, is_has_ugv_;
        double ori_x_, ori_y_, resolution_;
        double wx_min_, wx_max_, wy_min_, wy_max_;
        unsigned int size_x_, size_y_;
        double ground_height_min_, ground_height_max_, obstacle_height_max_;
        std::string global_frame_name_;
        double uav_height_;
        double laser_range_;
        bool is_pub_laser_;

    };
}

#endif //SIMPLE_MAP_2D_SIMPLE_MAP_2D_ROS_H
