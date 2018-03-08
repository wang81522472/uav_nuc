#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
double filter_resolution_;
double uav_fov_;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO_STREAM_ONCE("Receiving data! Filtering at resolution: " << filter_resolution_);
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (filter_resolution_, filter_resolution_, filter_resolution_);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

    sensor_msgs::PointCloud point_cloud_1;
    sensor_msgs::convertPointCloud2ToPointCloud(output, point_cloud_1);

    float y, z;
    for (auto it = point_cloud_1.points.begin(); it != point_cloud_1.points.end(); it++){
        y = it->y;
        z = it->z;
        if (y > (uav_fov_/2 + 0.07) || y < (-uav_fov_/2 + 0.07) || abs(z) > uav_fov_/2)
            point_cloud_1.points.erase(it);
    }
    sensor_msgs::convertPointCloudToPointCloud2(point_cloud_1, output);


  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud2_filter");
  ros::NodeHandle nh("~");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input_cloud", 1, cloud_cb);

  nh.param<double>("filter_resolution", filter_resolution_, 0.1);
    nh.param<double>("uav_fov", uav_fov_, 2.0);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_cloud", 1);

  // Spin
  ros::spin ();
}
