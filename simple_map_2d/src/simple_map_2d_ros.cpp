//
// Created by chengdaqian on 18-4-10.
//

#include <simple_map_2d/simple_map_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define TF_ZED_TO_BODY_SIM  0,-1,0,0,  -1,0,0,0,  0,0,-1,0,  0,0,0,1
#define TF_ZED_TO_BODY_REAL 0,0,-1,0,  0,-1,0,0,  -1,0,0,-0.05,  0,0,0,1

namespace simple_map_2d
{

    SimpleMap2DROS::SimpleMap2DROS(std::string name, std::string global_frame_name) :
        map_(NULL),
        private_nh_("~/" + name),
        global_frame_name_(global_frame_name)
    {
        loadParams();

        map_ = new SimpleMap2D(size_x_, size_y_, resolution_, ori_x_, ori_y_, NO_INFORMATION);

        if (is_sim_)
            tfZedToBody_ << TF_ZED_TO_BODY_SIM;
        else
            tfZedToBody_ << TF_ZED_TO_BODY_REAL;

        ugv_odom_sub_   = private_nh_.subscribe("ugv_odom",   5, &SimpleMap2DROS::ugv_odom_cb,   this);
        uav_odom_sub_   = private_nh_.subscribe("uav_odom",   5, &SimpleMap2DROS::uav_odom_cb,   this);
        ugv_sensor_sub_ = private_nh_.subscribe("ugv_sensor", 5, &SimpleMap2DROS::ugv_sensor_cb, this);
        uav_sensor_sub_ = private_nh_.subscribe("uav_sensor", 5, &SimpleMap2DROS::uav_sensor_cb, this);

        grid_map_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("simple_map", 5);
        grid_map_inflated_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("simple_map_inflated", 5);
        laser_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud",5);
        uav_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("uav_cloud", 5);
        setBoundaryObstacle();
    }

    void SimpleMap2DROS::pubMap() const
    {
        nav_msgs::OccupancyGrid grid_map, grid_map_inflated;
        grid_map_inflated.header.frame_id        = grid_map.header.frame_id        = global_frame_name_;
        grid_map_inflated.header.stamp           = grid_map.header.stamp           = ros::Time::now();
        grid_map_inflated.info.origin.position.x = grid_map.info.origin.position.x = ori_x_;
        grid_map_inflated.info.origin.position.y = grid_map.info.origin.position.y = ori_y_;
        grid_map_inflated.info.resolution        = grid_map.info.resolution        = resolution_;

        grid_map_inflated.info.width  = grid_map.info.width  = size_x_;
        grid_map_inflated.info.height = grid_map.info.height = size_y_;

        grid_map.data.clear();
        grid_map_inflated.data.clear();

        for (unsigned int idx = 0; idx < grid_map.info.width * grid_map.info.height; idx++){
            switch (map_->getCost(idx)){
                case LETHAL_OBSTACLE:
                    grid_map.data.push_back(100);
                    break;
                case FREE_SPACE:
                    grid_map.data.push_back(0);
                    break;
                case INSCRIBED_INFLATED_OBSTACLE:
                    grid_map.data.push_back(99);
                    break;
                case NO_INFORMATION:
                    grid_map.data.push_back(-1);
                    break;
                default:
                    ROS_WARN("Map Cell Value Iffy!!!");
                    grid_map.data.push_back(0);
                    break;
            }
            switch (map_->getCostInflated(idx)){
                case LETHAL_OBSTACLE:
                    grid_map_inflated.data.push_back(100);
                    break;
                case FREE_SPACE:
                    grid_map_inflated.data.push_back(0);
                    break;
                case INSCRIBED_INFLATED_OBSTACLE:
                    grid_map_inflated.data.push_back(99);
                    break;
                case NO_INFORMATION:
                    grid_map_inflated.data.push_back(-1);
                    break;
                default:
                    grid_map_inflated.data.push_back(char(1 + (97 * (map_->getCostInflated(idx) - 1)) / 251));
                    break;
            }
        }
        grid_map_pub_.publish(grid_map);
        grid_map_inflated_pub_.publish(grid_map_inflated);
    }

    void SimpleMap2DROS::loadParams(){

        nh_.param<bool>("/is_sim", is_sim_, true);
        nh_.param<bool>("/is_has_uav", is_has_uav_, true);
        nh_.param<bool>("/is_has_ugv", is_has_ugv_, true);
        private_nh_.param<bool>("is_pub_laser", is_pub_laser_, true);

        nh_.param<double>("/ugv_tf_offset_x", ugv_tf_offset_x_, 0.0);
        nh_.param<double>("/ugv_tf_offset_y", ugv_tf_offset_y_, 0.0);
        nh_.param<double>("/uav_tf_offset_x", uav_tf_offset_x_, 0.0);
        nh_.param<double>("/uav_tf_offset_y", uav_tf_offset_y_, 0.0);
        nh_.param<double>("/uav_height", uav_height_, 2.0);

        nh_.param<double>("/ground_height_max", ground_height_max_, 0.3);
        nh_.param<double>("/ground_height_min", ground_height_min_, -0.5);
        nh_.param<double>("/obstacle_height_max", obstacle_height_max_, 1.0);

        nh_.param<double>("/ground_range", laser_range_, 100.0);

        double bound_width;
        nh_.param<double>("/wx_min", wx_min_, -1.0);
        nh_.param<double>("/wx_max", wx_max_, 19.0);
        nh_.param<double>("/wy_min", wy_min_, -1.0);
        nh_.param<double>("/wy_max", wy_max_, 19.0);
        nh_.param<double>("/bound_width", bound_width, 3.0);
        nh_.param<double>("/resolution", resolution_, 0.1);

        ori_x_ = wx_min_ - bound_width;
        ori_y_ = wy_min_ - bound_width;

        size_x_ = (unsigned int)((wx_max_ - wx_min_ + 2 * bound_width) / resolution_);
        size_y_ = (unsigned int)((wy_max_ - wy_min_ + 2 * bound_width) / resolution_);
    }

    void SimpleMap2DROS::setBoundaryObstacle(){
        unsigned int mx0, my0, mx1, my1;

        map_->worldToMap(wx_min_, wy_min_, mx0, my0);
        map_->worldToMap(wx_min_, wy_max_, mx1, my1);
        map_->raytraceLine(LETHAL_OBSTACLE, mx0, my0, mx1, my1);
        map_->worldToMap(wx_max_, wy_max_, mx0, my0);
        map_->raytraceLine(LETHAL_OBSTACLE, mx0, my0, mx1, my1);
        map_->worldToMap(wx_max_, wy_min_, mx1, my1);
        map_->raytraceLine(LETHAL_OBSTACLE, mx0, my0, mx1, my1);
        map_->worldToMap(wx_min_, wy_min_, mx0, my0);
        map_->raytraceLine(LETHAL_OBSTACLE, mx0, my0, mx1, my1);
    }



    SimpleMap2DROS::~SimpleMap2DROS(){
        delete map_;
    }

    void SimpleMap2DROS::ugv_odom_cb(const nav_msgs::Odometry &ugv_odom) {
        //ROS_INFO("Enter UGV Odom CB!");

        ugv_pose_.position.x = ugv_odom.pose.pose.position.x - ugv_tf_offset_x_;
        ugv_pose_.position.y = ugv_odom.pose.pose.position.y - ugv_tf_offset_y_;

        ugv_pose_.orientation = ugv_odom.pose.pose.orientation;
        //ROS_INFO("Exit UGV Odom CB!");
    }

    void SimpleMap2DROS::uav_odom_cb(const nav_msgs::Odometry &uav_odom) {
        //ROS_INFO("Enter UAV Odom CB!");
        uav_pose_.position.x = uav_odom.pose.pose.position.x - uav_tf_offset_x_;
        uav_pose_.position.y = uav_odom.pose.pose.position.y - uav_tf_offset_y_;
	uav_pose_.position.z = uav_odom.pose.pose.position.z;
        uav_pose_.orientation = uav_odom.pose.pose.orientation;
        //ROS_INFO("Exit UGV Odom CB!");
    }

    void SimpleMap2DROS::ugv_sensor_cb(const sensor_msgs::LaserScan &ugv_scan){
        //ROS_INFO("Enter UGV Sensor CB!");
        if (!is_has_ugv_) //ignore sensor input if don't want UGV.
            return;

        if (ugv_scan.range_max < laser_range_)
            laser_range_ = ugv_scan.range_max;

        size_t n_pts = ugv_scan.ranges.size();
        Eigen::ArrayXXd ranges (n_pts, 2);
        Eigen::ArrayXXd co_sine_map(n_pts, 2);
        Eigen::ArrayXXd output (n_pts, 2);

        for (size_t i = 0; i < n_pts; ++i)
        {
            if (std::isfinite(ugv_scan.ranges[i]) && ugv_scan.ranges[i] < laser_range_)
            {
                ranges (i, 0) = (double) ugv_scan.ranges[i];
                ranges (i, 1) = (double) ugv_scan.ranges[i];
            }else{
                ranges (i, 0) = ranges(i, 1) = laser_range_;
            }
        }

        Eigen::Quaterniond q_eigen(ugv_pose_.orientation.w,  ugv_pose_.orientation.x, ugv_pose_.orientation.y, ugv_pose_.orientation.z);
        Eigen::Matrix3d Rgi = q_eigen.toRotationMatrix();
        double phi = asin(Rgi(2,1));
        double robot_angle = atan2(-Rgi(0,1)/cos(phi),Rgi(1,1)/cos(phi));

        double angle_min = ugv_scan.angle_min + robot_angle;
        double angle_max = ugv_scan.angle_max + robot_angle;

        // Spherical->Cartesian projection
        for (size_t i = 0; i < n_pts; ++i)
        {
            co_sine_map(i, 0) = cos (angle_min + (double) i * ugv_scan.angle_increment);
            co_sine_map(i, 1) = sin (angle_min + (double) i * ugv_scan.angle_increment);
        }
        output = ranges * co_sine_map;

        pubLaser(output, ugv_pose_.position.x, ugv_pose_.position.y);
        // process the map
        unsigned int mx, my, ugv_mx, ugv_my;
        map_->worldToMap(ugv_pose_.position.x, ugv_pose_.position.y, ugv_mx, ugv_my);

        for (size_t i = 0; i < n_pts; ++i){
            if (!(map_->worldToMap(output(i, 0) + ugv_pose_.position.x, output(i, 1) + ugv_pose_.position.y, mx, my)))
                continue;
            if (is_sim_) // keep obstacles in simulation
                map_->raytraceLineKeepObstacle(ugv_mx, ugv_my, mx, my);
            else
                map_->raytraceLine(FREE_SPACE, ugv_mx, ugv_my,mx,my);
            if (ranges(i,0) != laser_range_)
                map_->setCost(mx, my, LETHAL_OBSTACLE);
        }
        setBoundaryObstacle();
    }

    void SimpleMap2DROS::pubLaser(const Eigen::ArrayXXd& ranges, double center_x, double center_y)
    {
        pcl::PointCloud<pcl::PointXYZI> laser_cloud_viz;
        pcl::PointXYZI laser_point_viz(50);//initialize with 50 intensi      ty
        
        for (unsigned int idx = 0; idx < ranges.rows(); idx++){
            laser_point_viz.x = ranges(idx, 0) + center_x;
            laser_point_viz.y = ranges(idx, 1) + center_y;
            laser_cloud_viz.push_back(laser_point_viz);
        }

        //publish visualization point cloud
        sensor_msgs::PointCloud2 laser_viz_output;
        pcl::toROSMsg(laser_cloud_viz, laser_viz_output);
        laser_viz_output.header.frame_id = global_frame_name_;
        laser_viz_output.header.stamp = ros::Time::now();
        laser_cloud_pub_.publish(laser_viz_output);

        ROS_INFO("Time flag viz");
    }

    void SimpleMap2DROS::uav_sensor_cb(const sensor_msgs::PointCloud2 &uav_points) {
        //ROS_INFO("Enter UAV Sensor CB!");
        if (!is_has_uav_) //ignore sensor input if don't want UAV
            return;

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(uav_points, pcl_pc2);
        pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
        pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);

        unsigned int cloud_size = pcl_cloud.points.size();

        // translate points into global frame: zedToBody, bodyToWorld
        Eigen::MatrixX4f points_mat(cloud_size, 4), points_global(cloud_size, 4);
        Eigen::Matrix4f tf_b2w_;
        Eigen::Quaternionf q_eigen(uav_pose_.orientation.w, uav_pose_.orientation.x, uav_pose_.orientation.y, uav_pose_.orientation.z);


        tf_b2w_.block(0,0,3,3) = q_eigen.toRotationMatrix();
        tf_b2w_.block(0,3,4,1) << uav_pose_.position.x, uav_pose_.position.y, uav_pose_.position.z + uav_height_, 1.0;
        for (unsigned int i = 0; i < cloud_size; i++){
            points_mat.block(i, 0, 1, 4) << pcl_cloud.points[i].x,pcl_cloud.points[i].y,pcl_cloud.points[i].z,1.0;
        }
        points_global = points_mat * (tfZedToBody_.transpose()) * (tf_b2w_.transpose());

        // process the map
        unsigned int mx, my;

        for (unsigned int i = 0; i < cloud_size; i++){
            if (points_global(i,2) < ground_height_min_ || points_global(i,2) > obstacle_height_max_)
                continue;

            if (!(map_->worldToMap(points_global(i,0), points_global(i,1), mx, my)))
                continue; // abandon the point if out of bound
            if( Eigen::Vector2d(ugv_pose_.position.x-points_global(i,0),ugv_pose_.position.y-points_global(i,0)).norm()<0.5)
                continue;

            if (points_global(i,2) < ground_height_max_){
                map_->setCost(mx, my, FREE_SPACE);
            }else{
                map_->setCost(mx, my, LETHAL_OBSTACLE);
            }
        }
        //OS_INFO("Exit UAV Sensor CB!");
        setBoundaryObstacle();
        pubCloud(points_global);
    }

    void SimpleMap2DROS::pubCloud(const Eigen::MatrixX4f& points){
        ROS_INFO("Enter Cloud Viz");
        pcl::PointCloud<pcl::PointXYZI> uav_cloud_viz;
        pcl::PointXYZI uav_point_viz(50);//initialize with 50 intensi      ty
        
        for (unsigned int idx = 0; idx < points.rows(); idx++){
            uav_point_viz.x = points(idx,0);
            uav_point_viz.y = points(idx,1);
            uav_point_viz.z = points(idx,2);
            uav_cloud_viz.push_back(uav_point_viz);
        }

        //publish visualization point cloud
        sensor_msgs::PointCloud2 uav_viz_output;
        pcl::toROSMsg(uav_cloud_viz, uav_viz_output);
        uav_viz_output.header.frame_id = global_frame_name_;
        uav_viz_output.header.stamp = ros::Time::now();
        uav_cloud_pub_.publish(uav_viz_output);

        ROS_INFO("Time flag viz");
        ROS_INFO("Enter Cloud Viz");
    }

}
