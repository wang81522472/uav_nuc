//
// Created by chengdaqian on 18-4-10.
//

#ifndef SIMPLE_MAP_2D_LASER_PROC_H
#define SIMPLE_MAP_2D_LASER_PROC_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace laser_proc
{
    /**
     * @brief transform LaserScan into sensor_msgs::PointCloud2, frame remains the same.
     * @param scan_in
     * @param cloud_out
     */
    void transformLaserScanToPointCloud(const sensor_msgs::LaserScan &scan_in, sensor_msgs::PointCloud2 &cloud_out);

    /**
     * @brief transform LaserScan into pcl::PointCloud<pcl::PointXYZ>, frame remains the same.
     * @param scan_in
     * @param pcl_cloud_out
     */
    void transformLaserScanToPointCloudXYZ(const sensor_msgs::LaserScan &scan_in,
                                           pcl::PointCloud<pcl::PointXYZ> &pcl_cloud_out);
}

#endif //SIMPLE_MAP_2D_LASER_PROC_H
