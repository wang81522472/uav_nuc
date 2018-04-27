//
// Created by chengdaqian on 18-4-10.
//

#include <simple_map_2d/laser_proc.h>

namespace laser_proc
{
    void transformLaserScanToPointCloud(const sensor_msgs::LaserScan &scan_in, sensor_msgs::PointCloud2 &cloud_out)
    {
        size_t n_pts = scan_in.ranges.size();
        Eigen::ArrayXXd ranges (n_pts, 2);
        Eigen::ArrayXXd output (n_pts, 2);

        // Get the ranges into Eigen format
        for (size_t i = 0; i < n_pts; ++i)
        {
            ranges (i, 0) = (double) scan_in.ranges[i];
            ranges (i, 1) = (double) scan_in.ranges[i];
        }

        // Check if our existing co_sine_map is valid
        if (co_sine_map_.rows () != (int)n_pts || angle_min_ != scan_in.angle_min || angle_max_ != scan_in.angle_max )
        {
            ROS_DEBUG ("[projectLaser] No precomputed map given. Computing one.");
            co_sine_map_ = Eigen::ArrayXXd (n_pts, 2);
            angle_min_ = scan_in.angle_min;
            angle_max_ = scan_in.angle_max;
            // Spherical->Cartesian projection
            for (size_t i = 0; i < n_pts; ++i)
            {
                co_sine_map_ (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
                co_sine_map_ (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
            }
        }

        output = ranges * co_sine_map_;

        // Set the output cloud accordingly
        cloud_out.header = scan_in.header;
        cloud_out.height = 1;
        cloud_out.width  = scan_in.ranges.size ();
        cloud_out.fields.resize (3);
        cloud_out.fields[0].name = "x";
        cloud_out.fields[0].offset = 0;
        cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_out.fields[0].count = 1;
        cloud_out.fields[1].name = "y";
        cloud_out.fields[1].offset = 4;
        cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_out.fields[1].count = 1;
        cloud_out.fields[2].name = "z";
        cloud_out.fields[2].offset = 8;
        cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_out.fields[2].count = 1;

        // Define 4 indices in the channel array for each possible value type
        int idx_intensity = -1, idx_index = -1, idx_distance = -1, idx_timestamp = -1, idx_vpx = -1, idx_vpy = -1, idx_vpz = -1;

        //now, we need to check what fields we need to store
        int offset = 12;

        int field_size = cloud_out.fields.size();
        cloud_out.fields.resize(field_size + 1);
        cloud_out.fields[field_size].name = "intensity";
        cloud_out.fields[field_size].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_out.fields[field_size].offset = offset;
        cloud_out.fields[field_size].count = 1;
        offset += 4;
        idx_intensity = field_size;

        int field_size = cloud_out.fields.size();
        cloud_out.fields.resize(field_size + 1);
        cloud_out.fields[field_size].name = "index";
        cloud_out.fields[field_size].datatype = sensor_msgs::PointField::INT32;
        cloud_out.fields[field_size].offset = offset;
        cloud_out.fields[field_size].count = 1;
        offset += 4;
        idx_index = field_size;


        cloud_out.point_step = offset;
        cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
        cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
        cloud_out.is_dense = false;

        unsigned int count = 0;
        for (size_t i = 0; i < n_pts; ++i)
        {
            //check to see if we want to keep the point
            const float range = scan_in.ranges[i];
            if (range < scan_in.range_max && range >= scan_in.range_min)
            {
                float *pstep = (float*)&cloud_out.data[count * cloud_out.point_step];

                // Copy XYZ
                pstep[0] = output (i, 0);
                pstep[1] = output (i, 1);
                pstep[2] = 0;

                // Copy intensity
                if(idx_intensity != -1)
                    pstep[idx_intensity] = scan_in.intensities[i];

                //Copy index
                if(idx_index != -1)
                    ((int*)(pstep))[idx_index] = i;

                //make sure to increment count
                ++count;
            }

        }
        //resize if necessary
        cloud_out.width = count;
        cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
        cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
    }

    void transformLaserScanToPointCloudXYZ(const sensor_msgs::LaserScan &scan_in,
                                           pcl::PointCloud<pcl::PointXYZ> &pcl_cloud_out)
    {
        sensor_msgs::PointCloud2 sensor_cloud;
        transformLaserScanToPointCloud(scan_in, sensor_cloud);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(cloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud_out);
    }

}