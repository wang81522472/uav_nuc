//
// Created by chengdaqian on 18-4-15.
//

#ifndef DUET_EXPLORATION_HARMONIC_DUET_H
#define DUET_EXPLORATION_HARMONIC_DUET_H

#include <ros/ros.h>
#include <simple_map_2d/simple_map_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <duet_exploration/Frontier.h>
#include <duet_exploration/FrontierCell.h>
#include <duet_exploration/MultiArrayWithHeader.h>
#include <duet_exploration/path_search.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <simple_map_2d/map_inflation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "duet_exploration/dataType.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"

namespace duet_exploration
{
    using std::ofstream;
    class DuetExploration
    {
    public:
        DuetExploration(std::string node_name, std::string global_frame_name);
        //~DuetExploration();

        bool explore_main_loop();

        bool compAirGoalInfoGain(double wx, double wy, double& info_gain);

    //private:
        void rosMessageContainerInit();

        void copyMapsData();

        double countProgress(ofstream &time_file, ofstream &progress_file);

        void uav_odom_call_back(const nav_msgs::Odometry &uav_odom);

        void ugv_odom_call_back(const nav_msgs::Odometry ugv_odom);

        void visualize_field(const std::vector<double> &field, nav_msgs::OccupancyGrid &grid_viz, ros::Publisher &pub);
        void visCorridor(std::vector<Cube2D> corridor);

        bool checkInBound(unsigned int idx);

        bool procPickPubUAVGoal();

        bool visualize_frontier();

        //****** Harmonic Field Related functions ********* //
        bool initCharmapInfo();

        void initHarmonicFieldUGV();

        void getBfsSeq();

        void iterUpdate(std::vector<double>& harmonic_field, double& update_change, unsigned int robot_type);

        bool iterUpdateBFS(std::vector<double>& harmonic_field, double& update_change);

        bool compFieldGradient(const std::vector<double> &field, std::vector<double> &gradient, unsigned int robot_type);

        bool gradDescend(const std::vector<double> &gradient, unsigned int step_num, unsigned int robot_type);

        bool pubGrad(const std::vector<double> &gradient, ros::Publisher &pub);

        bool utilDiscount();

        bool motion_primitive();

        // ROS NodeHandles
        ros::NodeHandle nh_;
        ros::NodeHandle global_nh_;

        // ROS Subscribers
        ros::Subscriber uav_odom_sub_;
        ros::Subscriber ugv_odom_sub_;

        // ROS Publishers
        ros::Publisher corridor_vis_pub_;

        ros::Publisher uav_tree_viz_pub_;
        ros::Publisher uav_tree_point_viz_pub_;
        ros::Publisher uav_waypts_pub_;

        ros::Publisher frontier_cloud_pub_;
        ros::Publisher gridmap_no_inflation_pub_;
        ros::Publisher ugv_traj_pub_, ugv_traj_viz_pub_, ugv_checkTraj_vis_pub_;
        ros::Publisher harmonic_field_ugv_viz_pub_, harmonic_field_uav_fronA_viz_pub_, harmonic_field_uav_fronB_viz_pub_;
        ros::Publisher ugv_path_viz_pub_, uav_path_viz_pub_;
        ros::Publisher ugv_grad_viz_pub_, uav_grad_viz_pub_, uav_grad_fronA_viz_pub_, uav_grad_fronB_viz_pub_;
        ros::Publisher ugv_grad_pub_, uav_grad_pub_;
        ros::Publisher uav_goal_pub_;

        // System params
        bool is_sim_;
        bool is_has_uav_, is_has_ugv_;

        int min_frontier_size_;

        double ground_range_;
        double UAV_fov_x_, UAV_fov_y_; // length of FOV in meters
        double obstacle_height_;
        double uav_height_;
        double wx_min_, wx_max_, wy_min_, wy_max_;
        double update_duration_;
        double uav_tf_offset_x_, uav_tf_offset_y_, ugv_tf_offset_x_, ugv_tf_offset_y_;

        // Algorithm params
        double util_discnt_; // between 0 and 1
        double update_precision_;
        double uav_fronB_harmonic_;
        double uav_fronA_harmonic_;
        double uav_fronA_frontier_, uav_fronB_frontier_;
        int relaxation_method_;
        double sor_w_;
        bool is_iter_bfs_;

        // Data containers
        std::vector<unsigned char> charmap_no_inflation_, charmap_with_inflation_;
        std::vector<double> charmap_info_; // contains frontierA (size), frontierB(-1), free_space(0), obstacle information(-2)
        std::vector<double> ugv_coeff_vec_, uav_coeff_vec;//ugv_coeff contains frontier size, uav
        std::vector<double> harmonic_field_ugv_, harmonic_field_uav_fronA_, harmonic_field_uav_fronB_;
        std::vector<double> harmonic_grad_ugv_, harmonic_grad_uav_fronA_, harmonic_grad_uav_fronB_, harmonic_grad_uav_; // contains gradient direction in rads
        std::vector<unsigned int> new_ugv_boundary_idx_vec_, bfs_update_seq_;
        std::string node_name_, global_frame_name_;

        std::list<Frontier> frontierA_list_;
        std::list<FrontierCell> frontierA_cell_list_, frontierB_cell_list_;
        nav_msgs::OccupancyGrid grid_map_no_inflation_;
        nav_msgs::OccupancyGrid harmonic_field_ugv_viz_, harmonic_field_uav_fronA_viz_, harmonic_field_uav_fronB_viz_;

        geometry_msgs::PoseStamped uav_pose_, ugv_pose_;
        geometry_msgs::PointStamped uav_goal_;
        geometry_msgs::Twist uav_vel_;

        nav_msgs::Odometry ugv_odom_, uav_odom_;

        visualization_msgs::Marker ugv_path_viz_, uav_path_viz_;
        visualization_msgs::Marker ugv_grad_viz_, uav_grad_fronA_viz_, uav_grad_fronB_viz_, uav_grad_viz_;
        std::vector<unsigned int> ugv_path_idx_;

        unsigned int map_size_, size_x_, size_y_;
        double resolution_;

        // flags
        bool is_rcv_uav_odom_, is_rcv_ugv_odom_;

        visualization_msgs::MarkerArray cube_vis;
        // Map pointer
        boost::shared_ptr<simple_map_2d::SimpleMap2DROS> simple_map_;
        boost::shared_ptr<simple_map_2d::MapInflation>   map_inflation_;
    };
}

#endif //DUET_EXPLORATION_HARMONIC_DUET_H
