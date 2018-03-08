#ifndef AIR_GROUND_SIM_EXPLORE_LAYER_H_
#define AIR_GROUND_SIM_EXPLORE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Polygon.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/FrontierCell.h>
#include <frontier_exploration/MultiArrayWithHeader.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/BlacklistPoint.h>
#include <frontier_exploration/GetNextAirGroundFrontier.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace frontier_exploration
{

/**
 * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
 * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
 * and processes costmap to find next frontier to explore.
 */
class AirGroundSimExploreLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    AirGroundSimExploreLayer();
    ~AirGroundSimExploreLayer();

    double getGroundInfoGain(double wx, double wy);

    bool getExpA(double vx, double vy, double omega, unsigned int time_idx, Eigen::MatrixXd& mat_F) const;

    bool get_integral_ExpA(double vx, double vy, double omega, unsigned int time_idx, Eigen::MatrixXd &mat_F) const;

    void traj_viz(Eigen::MatrixXd &traj_mat, bool is_selected, double start_time, bool is_ugv);
    /**
     * @brief Loads default values and initialize exploration costmap.
     */
    virtual void onInitialize();

    /**
     * @brief Calculate bounds of costmap window to update
     */
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* polygon_min_x, double* polygon_min_y, double* polygon_max_x,
                              double* polygon_max_y);

    /**
     * @brief Update requested costmap window
     */
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized()
    {
        return true;
    }

    /**
     * @brief Match dimensions and origin of parent costmap
     */
    virtual void matchSize();

    /**
     * @brief Reset exploration progress
     */
    virtual void reset();
    
    /**
     * @brief Future define for visualization_msgs::Marker::DELETEALL. Constant is not defined in ROS Indigo, but functionality is implemented
     */
    static const int DELETEALL = 3;

        /**
      * @brief function to compute ground information gain
      * @param goal_candidate
      * @param frontierA_cell_list
      * @return false if visible frontierA cells < 3
      */
    bool computeInfoGainAndScoreGround(FrontierCell &goal_candidate, std::list<FrontierCell> &frontierA_cell_list);

    /**
     * @brief function to compute air information gain.
     * @param goal_candidate
     * @param frontierA_info_vec Pass in frontierA vector, which has the size of size_x_ * size_y_.
     *        To generate such vector, please see example in getNextAirFrontier func.
     * @param frontierB_info_vec Same as above
     * @param frontierB_flag
     * @return
     */
    bool computeInfoGainAndScoreAir(FrontierCell &goal_candidate, std::vector<double> &frontierA_info_vec,
                                    std::vector<double> &frontierB_info_vec, bool frontierB_flag);

    bool computeInfoGainAndScoreAir(FrontierCell &goal_candidate);

    costmap_2d::LayeredCostmap* getLayeredCostmapPtr(){
        return layered_costmap_;
    };

    bool getStateIndex(double vx, double vy, double omega, unsigned int &i, unsigned int &j, unsigned int &k) const;

protected:

    /**
     * @brief ROS Service wrapper for updateBoundaryPolygon
     * @param req Service request
     * @param res Service response
     * @return True on service success, false otherwise
     */
    bool updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res);

    /**
     * @brief Load polygon boundary to draw on map with each update
     * @param polygon_stamped polygon boundary
     * @return True if polygon successfully loaded, false otherwise
     */
    bool updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped);

    /**
     * @brief ROS Service wrapper for getNextFrontier
     * @param req Service request
     * @param res Service response
     * @return True on service success, false otherwise
     */
    bool getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res);

    /**
     * @brief Search the costmap for next reachable frontier to explore
     * @param start_pose Pose from which to start search
     * @param next_frontier Pose of found frontier
     * @return True if a reachable frontier was found, false otherwise
     */
    bool getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier);

    //***************************************************************************
    //******************** below are air-ground functions ***********************
    //***************************************************************************
    /**
     * @brief ROS Service wrapper for getNextAirGroundFrontier
     * @param req Service request
     * @param res Service response
     * @return True on service success, false otherwise
     */
    bool getNextAirGroundFrontierService(frontier_exploration::GetNextAirGroundFrontier::Request &req, frontier_exploration::GetNextAirGroundFrontier::Response &res);

    /**
     * @brief Search the costmap for next frontier goal for ugv and uav
     * @param start_pose_ground      UGV_Pose from which to start search
     * @param start_pose_air         UAV_Pose from which to start search
     * @param next_frontier_ground   Pose of frontier for UGV
     * @param next_frontier_air      Pose of frontier for UAV
     * @return  True if both goals are found
     */
    bool getNextAirGroundFrontier(geometry_msgs::PoseStamped start_pose_ground, geometry_msgs::PoseStamped start_pose_air, geometry_msgs::PoseStamped &next_frontier_ground, geometry_msgs::PoseStamped &next_frontier_air);
    
    /**
     * @brief Search the goal candidate for ugv and uav
     * @param goal_candidate_ground_list      UGV candidate list
     * @param goal_candidate_air_list         UAV candidate list
     * @param d_omega                         omega difference
     * @param time_step
     * @param next_frontier_ground
     * @param next_frontier_air
     * @return
     */
    void getCandidateList(std::list<FrontierCell> &goal_candidate_ground_list, std::list<FrontierCell> &goal_candidate_air_list, double d_omega, double time_step, geometry_msgs::PoseStamped &start_pose_ground, geometry_msgs::PoseStamped &start_pose_air);

    /**
     * @brief Search the costmap for next frontier goal for UGV
     * @param start_pose_ground
     * @param next_frontier_ground
     * @param frontierA_cell_list  List of frontierA cells
     * @return
     */
    bool getNextGroundFrontier(geometry_msgs::PoseStamped start_pose_ground, geometry_msgs::PoseStamped &next_frontier_ground, std::list<FrontierCell>& frontierA_cell_list, std::list<FrontierCell>& goal_candidate_list);

    /**
     * @brief Search the costmap for next frontier goal for UAV
     * @param start_pose_air
     * @param next_frontier_air
     * @param frontierA_cell_list  list of frontierA cells
     * @param frontierB_cell_list  list of frontierB cells
     * @return
     */
    bool getNextAirFrontier(geometry_msgs::PoseStamped start_pose_air, geometry_msgs::PoseStamped &next_frontier_air, std::list<FrontierCell>& frontierA_cell_list, std::list<FrontierCell>& frontierB_cell_list, std::list<FrontierCell>& goal_candidate_list, bool frontierB_flag);

    /**
     * @brief Ray trace free. Used in UGV info gain count.
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @return
     */
    bool raytraceFree(int x0, int y0, int x1, int y1);

    bool bresenham2D_raytraceFree(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length);

    /**
     * @brief Ray trace free 2.5D version. Used in UAV info gain count.
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     * @return
     */
    bool raytraceFree3D(int x0, int y0, int x1, int y1);

    bool bresenham2D_raytraceFree3D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length);

    /**
     * @brief Transform pose into costmap's frame_id if not the same.
     * @param pose
     * @return True if transformation is successful
     */
    bool transformPose(geometry_msgs::PoseStamped& pose);

    bool transformOdom(nav_msgs::Odometry &odom, std::string twist_frame);

    /**
     * @brief ROS Service wrapper for adding a point to the frontier blacklist
     * @param req Service request
     * @param res Service response
     * @return Always true
     */
    bool blacklistPointService(frontier_exploration::BlacklistPoint::Request &req, frontier_exploration::BlacklistPoint::Response &res);

    /**
     * @brief ROS Service wrapper for clearing the frontier blacklist
     * @param req Service request
     * @param res Service response
     * @return Always true
     */
    bool clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    void visualize(std::list<FrontierCell> &frontierA_cell_list, std::list<FrontierCell> &frontierB_cell_list,geometry_msgs::PoseStamped &next_frontier_ground, geometry_msgs::PoseStamped &next_frontier_air);

    void visualize_info_gain(std::list<FrontierCell> &frontierA_cell_list);








private:

    /**
     * @brief Update the map with exploration boundary data
     * @param master_grid Reference to master costmap
     */
    void mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

    void prepFrontierACell(std::list<Frontier> &frontierA_list, std::list<FrontierCell> &frontierA_cell_list);

    void prepFrontierACell(std::list<Frontier> &frontierA_list, std::list<FrontierCell> &frontierA_cell_list,
                           geometry_msgs::PoseStamped start_pose_ground, geometry_msgs::PoseStamped start_pose_air,
                           bool compute_ground_cost = true, bool compute_air_cost = true);

    void prepFrontierBCell(std::list<FrontierCell> &frontierB_cell_list, geometry_msgs::PoseStamped start_pose_air);

    void genCharmapNoInflation();

    void pubGridmapNoInflation();

    void countProgress();
    
    void uav_odom_call_back(const nav_msgs::Odometry &uav_odom);
    
    void ugv_odom_call_back(const nav_msgs::Odometry ugv_odom);

    void genInfoVec(std::list<FrontierCell> &frontierA_cell_list, std::list<FrontierCell> &frontierB_cell_list);

    void init_state_mat_init();




    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    ros::ServiceServer polygonService_;
    ros::ServiceServer frontierService_;
    ros::ServiceServer blacklistPointService_;
    ros::ServiceServer clearBlacklistService_;
    ros::ServiceServer airGroundFrontierService_;
    geometry_msgs::Polygon polygon_;
    tf::TransformListener tf_listener_;

    ros::Publisher frontier_cloud_pub;
    ros::Publisher frontier_cell_pub_;
    ros::Publisher blacklist_marker_pub_;

    bool configured_, marked_;

    std::list<geometry_msgs::Point> blacklist_;

    std::string global_frame_;
    std::string frontier_travel_point_;
    bool resize_to_boundary_;
    int min_frontier_size_;
    double blacklist_radius_;

    double utility_discount_; // between 0 and 1
    double score_information_gain_coeff_;
    double score_frontier_size_coeff_;
    double score_traverse_cost_coeff_;

    double ground_range_;
    double air_range_;

    // weights of components of UAVs' information gain, add up to be 1.0
    double info_weight_frontierA_;
    double info_weight_frontierB_;
    double info_weight_unknown_;

    // length of fov in meters
    double UAV_fov_x;
    double UAV_fov_y;

    double frontier_search_boundary_min_;
    double frontier_search_boundary_max_;

    double obstacle_height_;
    double uav_height_;
    
    double ugv_vx, ugv_vy, uav_vx, uav_vy;

    bool frontierB_flag_;

    std::vector<unsigned char> charmap_no_inflation_;
    std::vector<double> information_gain_vec_;


    // frontier viz for debug
    ros::Publisher frontierA_visible_pub_, frontierA_node_pub_;
    ros::Publisher ugv_mat_pub_, uav_mat_pub_;
    ros::Publisher ugv_goal_point_pub_;
    ros::Publisher uav_tree_viz_pub_, ugv_tree_viz_pub_;
    ros::Publisher tree_edge_viz_pub_;

    ros::Subscriber uav_odom_sub_;
    ros::Subscriber ugv_odom_sub_;

    std::vector<double> frontierA_info_vec_;
    std::vector<double> frontierB_info_vec_;
    
    Eigen::VectorXd uav_pos;
    Eigen::VectorXd ugv_pos;

    nav_msgs::Odometry ugv_odom_;
    nav_msgs::Odometry uav_odom_;

    bool uav_flag_odom_, ugv_flag_odom_;

    Eigen::Matrix<double,6,6> ****state_mat_init_;
    Eigen::Matrix<double,6,6> ****state_mat_integral_init_;

    ros::Publisher gridmap_no_inflation_pub_;
    ros::Publisher traj_candidates_pub_, selected_traj_pub_;
    ros::Publisher ugv_acc_pub_;

    nav_msgs::OccupancyGrid grid_map_no_inflation_;
    visualization_msgs::Marker selected_marker_, tree_traj_marker_;

    double wx_min, wx_max, wy_min, wy_max;
    double path_search_rho_;

    bool flag_uav_, flag_ugv_;

    bool flag_is_sim_;
pcl::PointCloud<pcl::PointXYZI> frontierA_visible_viz;
    double infogain_range_;
};

}
#endif
