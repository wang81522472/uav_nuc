#include <frontier_exploration/air_ground_sim_explore_layer.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>

#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/BlacklistPoint.h>
#include <frontier_exploration/frontier_search.h>
#include <frontier_exploration/geometry_tools.h>

#include <uav_costmap_obstacle_layer/uav_obstacle_layer.h>
#include <frontier_exploration/path_search.h>



#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_listener.h>



#define DISC_COEF 41
#define ugv_max_vx 1.0
#define ugv_max_vy 1.0
#define ugv_max_omega 1.0
#define layer_num 4
#define set_UGV_A 0,1,0,0,0,0, 0,0,0,-omega,0,-vy, 0,0,0,1,0,0, 0,omega,0,0,0,vx, 0,0,0,0,0,1, 0,0,0,0,0,0


const double time_step[layer_num] = {0.2, 1.0, 1.5, 2.0};

PLUGINLIB_EXPORT_CLASS(frontier_exploration::AirGroundSimExploreLayer, costmap_2d::Layer)

namespace frontier_exploration
{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;
    using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

    inline int fac(int k){
        return k<2 ? 1:k*fac(k-1);
    }

    Eigen::MatrixXd mat_pow(Eigen::MatrixXd base, unsigned int n){
        Eigen::MatrixXd id=Eigen::MatrixXd::Identity(base.rows(),base.cols());
        return n == 0 ? id : (base * mat_pow (base, n - 1));
    }

    AirGroundSimExploreLayer::AirGroundSimExploreLayer(){}

    AirGroundSimExploreLayer::~AirGroundSimExploreLayer(){
        polygonService_.shutdown();
        frontierService_.shutdown();
        delete dsrv_;
        dsrv_ = 0;

        for (unsigned int time_idx = 0; time_idx < layer_num; time_idx ++){
            for (unsigned int i = 0; i < DISC_COEF; i ++) {
                for (unsigned int j = 0; j < DISC_COEF; j ++) {
                    delete[] state_mat_init_[time_idx][i][j];
                    delete[] state_mat_integral_init_[time_idx][i][j];
                }
                delete[] state_mat_init_[time_idx][i];
                delete[] state_mat_integral_init_[time_idx][i];
            }
            delete[] state_mat_init_[time_idx];
            delete[] state_mat_integral_init_[time_idx];
        }
        delete[] state_mat_init_;
        delete[] state_mat_integral_init_;
    }

    void AirGroundSimExploreLayer::traj_viz(Eigen::MatrixXd &traj_mat, bool is_selected, double start_time, bool is_ugv){


	    double des_x = 0, des_y = 0;
        geometry_msgs::Point parent, child;
        parent.z = child.z = 0;
        Eigen::MatrixXd coef=traj_mat.block(0,0,traj_mat.rows(),traj_mat.cols()-2);
        Eigen::VectorXd T    = traj_mat.col(coef.cols() +1);
        Eigen::MatrixXd coef_x,coef_y;
        if (is_selected){
            selected_marker_.header.stamp = ros::Time::now();
            bool first=true;
            for(double dT = ros::Time::now().toSec();dT< T(T.rows()-1);dT+=0.1) {
                if(first){
                    parent.x=coef(0,0);
                    parent.y=coef(0,6);
                    if (is_ugv)
                        parent.z = 0.3;
                    else
                        parent.z = 2.0;
                    selected_marker_.points.push_back(parent);
                    first=false;
                }
                for (int i = 0; i < T.size(); i++) {
                    if (dT < T(i)) {
                        double tt = i > 0 ? dT - T(i - 1) : dT - start_time;

                        //std::cout<<"T(i-1): "<<T(i-1)<<'\n';

                        Eigen::Matrix<double, 1, 6> t_p;
                        t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);
                        Eigen::Matrix<double, 1, 6> t_v;
                        t_v << 0, 1, 2 * tt, 3 * pow(tt, 2), 4 * pow(tt, 3), 5 * pow(tt, 4);

                        Eigen::VectorXd coef_x;
                        Eigen::VectorXd coef_y;
                        coef_x = (coef.block(i, 0, 1, 6)).transpose();
                        coef_y = (coef.block(i, 6, 1, 6)).transpose();

                        des_x = t_p * coef_x;
                        des_y = t_p * coef_y;
                        child.x = des_x;
                        child.y = des_y;
                        parent.x=des_x;
                        parent.y=des_y;
                        if (is_ugv){
                            child.z = parent.z = 0.3;
                        }else{
                            child.z = parent.z = 2.0;
                        }
                        selected_marker_.points.push_back(child);
                        selected_marker_.points.push_back(parent);
                        break;
                    }
                }
            }

	    if (!selected_marker_.points.empty())
                selected_marker_.points.pop_back();


        }else{
            selected_marker_.header.stamp = ros::Time::now();
            for(int i=0; i<traj_mat.rows(); ++i){
                if(i==0) {
                    parent.x = traj_mat(i,0);
                    parent.y = traj_mat(i,1);
                    selected_marker_.points.push_back(parent);
                }
                else{
                    child.x = traj_mat(i,0);
                    child.y = traj_mat(i,1);
                    parent.x = traj_mat(i,0);
                    parent.y = traj_mat(i,1);
                    selected_marker_.points.push_back(child);
                    selected_marker_.points.push_back(parent);
                }
            }



	    if (!selected_marker_.points.empty())
                selected_marker_.points.pop_back();


        }

    }

    void AirGroundSimExploreLayer::onInitialize(){

        ROS_WARN("flag_ enter init!");

        ros::NodeHandle nh_("~/" + name_);
        ros::NodeHandle global_nh_;
        frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
        blacklist_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("blacklist", 5);
        frontier_cell_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("frontier_cells",5);
        uav_odom_sub_ = global_nh_.subscribe("uav_odom",1,&AirGroundSimExploreLayer::uav_odom_call_back,this);
        ugv_odom_sub_ = global_nh_.subscribe("ugv_odom",1,&AirGroundSimExploreLayer::ugv_odom_call_back,this);
        uav_mat_pub_ = nh_.advertise<frontier_exploration::MultiArrayWithHeader>("uav_mat",2);
        ugv_mat_pub_ = nh_.advertise<frontier_exploration::MultiArrayWithHeader>("ugv_mat",2);
        ugv_goal_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("ugv_goal_point",5);
        uav_tree_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("uav_path_tree",5);
        ugv_tree_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ugv_path_tree",5);
        tree_edge_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_edge_viz", 5);
        traj_candidates_pub_ = nh_.advertise<visualization_msgs::Marker>("traj_candidates_viz", 1);
        selected_traj_pub_   = nh_.advertise<visualization_msgs::Marker>("traj_selected_uav_viz", 1);
        ugv_acc_pub_ = nh_.advertise<geometry_msgs::Twist>("/ugv_acc", 5);
        
        configured_ = false;
        marked_ = false;

        bool explore_clear_space;
        nh_.param("explore_clear_space", explore_clear_space, true);
        if(explore_clear_space){
            default_value_ = NO_INFORMATION;
        }else{
            default_value_ = FREE_SPACE;
        }

        ROS_WARN("flag_  before match size!");

        matchSize();

        nh_.param<bool>("resize_to_boundary", resize_to_boundary_, true);
        nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "closest");
        nh_.param<int>("min_frontier_size", min_frontier_size_, 5);
        nh_.param<double>("utility_discount", utility_discount_, 0.5);
        nh_.param<double>("score_information_gain_coeff", score_information_gain_coeff_, 0.5);
        nh_.param<double>("score_frontier_size_coeff", score_frontier_size_coeff_, 0.2);
        nh_.param<double>("/ground_range", ground_range_, 8.0);
	nh_.param<double>("/infogain_range", infogain_range_, 3.0);
        //time_step[0] = ground_range_ / 40.0;
	//time_step[1] = ground_range_ / 8.0;
	//time_step[2] = ground_range_ / 8.0 * 3.0;
	//time_step[3] = ground_range_ / 8.0 * 5.0;	

        score_traverse_cost_coeff_ = 1.0 - score_frontier_size_coeff_ - score_information_gain_coeff_;// 3 coeffs add up to 1.0

        nh_.param<double>("info_weight_frontierA", info_weight_frontierA_, 0.1);
        nh_.param<double>("info_weight_frontierB", info_weight_frontierB_, 0.89);
        info_weight_unknown_ = 1.0 - info_weight_frontierB_ - info_weight_frontierA_;
        if (info_weight_unknown_ < 0){
            ROS_ERROR("Combined weight of frontierA and frontierB should not exceed 1.");
            info_weight_unknown_ = 0.0;
        }

        nh_.param<double>("UAV_fov_x", UAV_fov_x, 0.6);
        nh_.param<double>("UAV_fov_y", UAV_fov_y, 0.6);

        nh_.param<double>("obstacle_height", obstacle_height_, 0.6);
        nh_.param<double>("uav_height", uav_height_, 2.0);

        nh_.param<double>("/wx_min", wx_min, -1.0);
        nh_.param<double>("/wx_max", wx_max, 5.0);
        nh_.param<double>("/wy_min", wy_min, -1.0);
        nh_.param<double>("/wy_max", wy_max, 5.0);

        nh_.param<bool>("/flag_uav", flag_uav_, true);
        nh_.param<bool>("/flag_ugv", flag_ugv_, true);

	nh_.param<bool>("/flag_is_sim", flag_is_sim_, true);

        ROS_WARN_STREAM("wx_min" << wx_min << ", wx_max" << wx_max << ", wy_min" << wy_min << ", wy_max" << wy_max);

        ROS_WARN_STREAM("Ground range set to: " << ground_range_);
        nh_.param<double>("/path_search_rho", path_search_rho_, 1.0);
        ROS_WARN_STREAM("*** rho set to: " << path_search_rho_ );

        polygonService_ = nh_.advertiseService("update_boundary_polygon", &AirGroundSimExploreLayer::updateBoundaryPolygonService, this);
        frontierService_ = nh_.advertiseService("get_next_frontier", &AirGroundSimExploreLayer::getNextFrontierService, this);
        blacklistPointService_ = nh_.advertiseService("blacklist_point", &AirGroundSimExploreLayer::blacklistPointService, this);
        clearBlacklistService_ = nh_.advertiseService("clear_blacklist", &AirGroundSimExploreLayer::clearBlacklistService, this);
        airGroundFrontierService_ = nh_.advertiseService("get_next_air_ground_frontier", &AirGroundSimExploreLayer::getNextAirGroundFrontierService, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &AirGroundSimExploreLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // frontier viz for debug
        frontierA_visible_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("frontierA_visible",5);
        frontierA_node_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("frontierA_node_visible", 5);

        uav_flag_odom_ = ugv_flag_odom_ = false;


        ROS_INFO("Starting to init state matrix...");
        state_mat_init_ = new Eigen::Matrix<double,6,6>***[layer_num];
        state_mat_integral_init_ = new Eigen::Matrix<double,6,6>***[layer_num];
        unsigned int cnt_addr = 0;
        for (unsigned int time_idx = 0; time_idx < layer_num; time_idx++){
            state_mat_init_[time_idx] = new Eigen::Matrix<double,6,6>**[DISC_COEF];
            state_mat_integral_init_[time_idx] = new Eigen::Matrix<double,6,6>**[DISC_COEF];
            for (unsigned int i = 0; i < DISC_COEF; i ++) {
                state_mat_init_[time_idx][i] = new Eigen::Matrix<double,6,6>*[DISC_COEF];
                state_mat_integral_init_[time_idx][i] = new Eigen::Matrix<double,6,6>*[DISC_COEF];
                for (unsigned int j = 0; j < DISC_COEF; j ++) {
                    state_mat_init_[time_idx][i][j] = new Eigen::Matrix<double,6,6>[DISC_COEF];
                    state_mat_integral_init_[time_idx][i][j] = new Eigen::Matrix<double,6,6>[DISC_COEF];
                    cnt_addr++;
                }
            }
        }
        ROS_INFO_STREAM("Assigned mem to init state matrix: " << cnt_addr << " total: " <<  DISC_COEF * DISC_COEF * layer_num);

        init_state_mat_init();
        ROS_INFO("Inited value of state matrix...");

        gridmap_no_inflation_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gridmap_no_inflation", 1);

        grid_map_no_inflation_.header.frame_id        = layered_costmap_->getGlobalFrameID();
        grid_map_no_inflation_.header.stamp           = ros::Time::now();
        grid_map_no_inflation_.info.origin.position.x = layered_costmap_->getCostmap()->getOriginX();
        grid_map_no_inflation_.info.origin.position.y = layered_costmap_->getCostmap()->getOriginY();
        grid_map_no_inflation_.info.resolution        = layered_costmap_->getCostmap()->getResolution();

        selected_marker_.header.frame_id = layered_costmap_->getGlobalFrameID();
        selected_marker_.action = visualization_msgs::Marker::ADD;
        selected_marker_.pose.orientation.w = 1.0;
        selected_marker_.id = 0;
        selected_marker_.type = visualization_msgs::Marker::LINE_LIST;
        selected_marker_.scale.x = 0.02;
        selected_marker_.color.b = selected_marker_.color.a = 1.0;

        tree_traj_marker_.header.frame_id = layered_costmap_->getGlobalFrameID();
        tree_traj_marker_.action = visualization_msgs::Marker::ADD;
        tree_traj_marker_.pose.orientation.w = 1.0;
        tree_traj_marker_.id = 0;
        tree_traj_marker_.type = visualization_msgs::Marker::LINE_LIST;
        tree_traj_marker_.scale.x = 0.02;
        tree_traj_marker_.color.b = tree_traj_marker_.color.a = 1.0;

    }

    void AirGroundSimExploreLayer::init_state_mat_init() {
        double vx, vy, omega;
        unsigned int time_idx, i, j, k;

        for (time_idx = 0; time_idx < layer_num; time_idx++)
            for (i = 0; i < DISC_COEF; i ++)
                for (j = 0; j < DISC_COEF; j++)
                    for (k = 0; k < DISC_COEF; k++){

                        vx = -ugv_max_vx + i * (2 * ugv_max_vx / (DISC_COEF - 1));
                        vy = -ugv_max_vy + j * (2 * ugv_max_vy / (DISC_COEF - 1));
                        omega = -ugv_max_omega + k * (2 * ugv_max_omega / (DISC_COEF));

                        Eigen::Matrix<double,6,6> A = Eigen::MatrixXd::Zero(6,6);
                        A << set_UGV_A;

                        state_mat_integral_init_[time_idx][i][j][k] = Eigen::MatrixXd::Zero(6,6);
                        for(int order = 0; order < 5; ++order){
                            state_mat_integral_init_[time_idx][i][j][k] = state_mat_integral_init_[time_idx][i][j][k]
                                                                          + std::pow(time_step[time_idx],order+1)/fac(order+1) * mat_pow(A,order);
                        }
                        state_mat_init_[time_idx][i][j][k] = (A * time_step[time_idx]).exp();
                    }

        ROS_WARN_STREAM("Finished init state matrix");
    }

    bool AirGroundSimExploreLayer::getStateIndex(double vx, double vy, double omega,
                                                 unsigned int &i, unsigned int &j, unsigned int &k) const{
        if (std::abs(vx) > ugv_max_vx || std::abs(vy) > ugv_max_vy || std::abs(omega) > ugv_max_omega){
            //ROS_WARN("speed or angular speed out of boundary!");
            return false;
        }
        i = (unsigned int)((vx + ugv_max_vx) / (2 * ugv_max_vx / (DISC_COEF - 1)));
        j = (unsigned int)((vy + ugv_max_vy) / (2 * ugv_max_vy / (DISC_COEF - 1)));
        k = (unsigned int)((omega + ugv_max_omega) / (2 * ugv_max_omega / (DISC_COEF - 1)));
        return true;
    }

    bool AirGroundSimExploreLayer::getExpA(double vx, double vy, double omega, unsigned int time_idx, Eigen::MatrixXd &mat_F) const{
        if (std::abs(vx) > ugv_max_vx || std::abs(vy) > ugv_max_vy || std::abs(omega) > ugv_max_omega){
            //ROS_WARN("speed or angular speed out of boundary!");
            //ROS_WARN_STREAM("vx: " << vx << ", vy: " << vy << ", omega: " << omega);

            return false;
        }
        unsigned int i, j, k;
        getStateIndex(vx, vy, omega, i, j, k);
        mat_F = state_mat_init_[time_idx][i][j][k];
        return true;
    }

    bool AirGroundSimExploreLayer::get_integral_ExpA(double vx, double vy, double omega, unsigned int time_idx, Eigen::MatrixXd &mat_F) const{
        if (std::abs(vx) > ugv_max_vx || std::abs(vy) > ugv_max_vy || std::abs(omega) > ugv_max_omega){
            //ROS_WARN("speed or angular speed out of boundary!");
            //ROS_WARN_STREAM("vx: " << vx << ", vy: " << vy << ", omega: " << omega);
            return false;
        }
        unsigned int i, j, k;
        getStateIndex(vx, vy, omega, i, j, k);
        mat_F = state_mat_integral_init_[time_idx][i][j][k];
        return true;
    }

    void AirGroundSimExploreLayer::matchSize(){
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }


    void AirGroundSimExploreLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }

    bool AirGroundSimExploreLayer::getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res){
        return getNextFrontier(req.start_pose, res.next_frontier);
    }

    bool AirGroundSimExploreLayer::getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier){

        //wait for costmap to get marked with boundary
        ros::Rate r(10);
        while(!marked_){
            ros::spinOnce();
            r.sleep();
        }
        
        if(start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            //error out if no transform available
            if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                ROS_ERROR_STREAM("GetNextFrontier: Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
                return false;
            }
            geometry_msgs::PoseStamped temp_pose = start_pose;
            tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(),temp_pose,start_pose);
        }

        //initialize frontier search implementation
        FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()), min_frontier_size_, frontier_travel_point_,
                                      wx_min, wx_max, wy_min, wy_max);
        //get list of frontiers from search implementation
        std::list<Frontier> frontier_list = frontierSearch.searchFrontierAFrom(start_pose.pose.position);

        if(frontier_list.size() == 0){
            ROS_DEBUG("No frontiers found, exploration complete");
            return false;
        }

        //create placeholder for selected frontier
        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);
        int max;

        BOOST_FOREACH(Frontier frontier, frontier_list){
            //load frontier into visualization poitncloud
            frontier_point_viz.x = frontier.travel_point.x;
            frontier_point_viz.y = frontier.travel_point.y;
            frontier_cloud_viz.push_back(frontier_point_viz);

            //check if this frontier is the nearest to robot
            if (frontier.min_distance < selected.min_distance && !anyPointsNearby(frontier.travel_point, blacklist_, blacklist_radius_)){
                selected = frontier;
                max = frontier_cloud_viz.size()-1;
            }
        }

        if (std::isinf(selected.min_distance)) {
            ROS_DEBUG("No valid (non-blacklisted) frontiers found, exploration complete");
            return false;
        }

        //color selected frontier
        frontier_cloud_viz[max].intensity = 100;

        //publish visualization point cloud
        sensor_msgs::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontier_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub.publish(frontier_viz_output);

        //set goal pose to next frontier
        next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier.header.stamp = ros::Time::now();

        //
        next_frontier.pose.position = selected.travel_point;
        next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose.pose.position, next_frontier.pose.position) );
        return true;
    }

    // *************************************************************************
    // ************** New functions for air ground exploration *****************
    // *************************************************************************

    bool AirGroundSimExploreLayer::getNextAirGroundFrontierService(
            frontier_exploration::GetNextAirGroundFrontier::Request &req,
            frontier_exploration::GetNextAirGroundFrontier::Response &res)
    {
        ROS_WARN("Got a srv req!");
        return getNextAirGroundFrontier(req.start_pose_ground, req.start_pose_air,
                                        res.next_frontier_ground, res.next_frontier_air);
    }

    void AirGroundSimExploreLayer::visualize_info_gain(std::list<FrontierCell> &frontierA_cell_list) {

        frontierA_visible_viz.clear();

        std::list<FrontierCell> free_cell_list;
        unsigned int mx, my;
        double dist, max_IG = 0.0, wx, wy;
        for (unsigned idx = 0; idx < size_x_ * size_y_; idx++){
            information_gain_vec_[idx] = 0.0;
            FrontierCell fc;
            layered_costmap_->getCostmap()->indexToCells(idx, mx, my);
            layered_costmap_->getCostmap()->mapToWorld(mx, my, wx, wy);
            if (abs(wx - ugv_odom_.pose.pose.position.x) >= infogain_range_ || abs(wy - ugv_odom_.pose.pose.position.y) >= infogain_range_)
                continue;
            if (wx > wx_max || wx < wx_min || wy > wy_max || wy < wy_min)
                continue;
            if (layered_costmap_->getCostmap()->getCost(mx, my) == FREE_SPACE){
                fc.frontier_cell_point.x = mx;
                fc.frontier_cell_point.y = my;

                for (std::list<FrontierCell>::iterator j = frontierA_cell_list.begin(); j != frontierA_cell_list.end(); j++){
                    // if raytrace no obstacle
                    dist = sqrt(pow(mx - j->frontier_cell_point.x,2) + pow(my - j->frontier_cell_point.y,2));

                    if (dist <= ground_range_ / (layered_costmap_->getCostmap())->getResolution()) {

                        if(raytraceFree(int(mx), int(my), int(j->frontier_cell_point.x), int(j->frontier_cell_point.y))){

                            (fc.information_gain) += j->utility * log10(j->frontier_size) / dist;
                        }
                    }
                }
                free_cell_list.push_back(fc);
                if (fc.information_gain > max_IG)
                    max_IG = fc.information_gain;

            }
            information_gain_vec_[idx] = fc.information_gain;
        }
        double viz_wx, viz_wy;
        BOOST_FOREACH(FrontierCell fc, free_cell_list){
                        pcl::PointXYZI frontier_point_viz(fc.information_gain/max_IG * 100);
                        (layered_costmap_->getCostmap())->mapToWorld(
                                fc.frontier_cell_point.x, fc.frontier_cell_point.y,viz_wx, viz_wy);
                        frontier_point_viz.x = viz_wx;
                        frontier_point_viz.y = viz_wy;
                        frontierA_visible_viz.push_back(frontier_point_viz);
                    }

        //visualize visible fronA
        sensor_msgs::PointCloud2 frontierA_viz_output;
        pcl::toROSMsg(frontierA_visible_viz, frontierA_viz_output);
        frontierA_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontierA_viz_output.header.stamp = ros::Time::now();
        frontierA_visible_pub_.publish(frontierA_viz_output);

    }

    double AirGroundSimExploreLayer::getGroundInfoGain(double wx, double wy) {
        unsigned int mx, my, idx;
        if ((layered_costmap_->getCostmap())->worldToMap(wx, wy, mx, my)) {
            idx = (layered_costmap_->getCostmap())->getIndex(mx, my);
            return information_gain_vec_[idx];
        } else{
            return 0.0;
        }
    }

    bool AirGroundSimExploreLayer::getNextAirGroundFrontier(geometry_msgs::PoseStamped start_pose_ground,
                                                            geometry_msgs::PoseStamped start_pose_air,
                                                            geometry_msgs::PoseStamped &next_frontier_ground,
                                                            geometry_msgs::PoseStamped &next_frontier_air)
    {

        ROS_INFO("Enter getNextAirGroundFrontier Service");
        //wait for costmap to get marked with boundary
        ros::Rate r(2);
        while(!marked_){
            ros::spinOnce();
            r.sleep();
        }

        // transform poses
        //transformPose(start_pose_ground);
        //transformPose(start_pose_air);
        //the loop begin here

        double prev_front_x=0;
        double prev_front_y=0;
        bool uav_fail_flag=0;
        double loop_start_time;
        while(true) {
            loop_start_time = ros::Time::now().toSec();

            ros::spinOnce();

            // enabled_ = false; // No more illegal boundary change warning if uncomment this

            Eigen::MatrixXd uav_coef, ugv_coef;
            frontier_exploration::MultiArrayWithHeader ugv_mat, uav_mat;
            geometry_msgs::PointStamped ugv_goal_point;


            // get inflated_visited_ list from uav_obstacle_layer
            genCharmapNoInflation();
            pubGridmapNoInflation();
            ROS_INFO("Time flag Prep");

            //count exploration progress
            countProgress();

            //initialize frontier search implementation and search FrontierA and frontierB_cell
            FrontierSearch frontierSearch(*(layered_costmap_->getCostmap()), min_frontier_size_, frontier_travel_point_,
                                          wx_min, wx_max, wy_min, wy_max);
            std::list<Frontier> frontierA_list = frontierSearch.searchFrontierA(charmap_no_inflation_);
            std::list<FrontierCell> frontierB_cell_list = frontierSearch.searchFrontierBCell(charmap_no_inflation_);
            ROS_INFO("Time flag searched Frontier");
            if (frontierA_list.empty() && frontierB_cell_list.empty()) {
                ROS_WARN("No frontiers of any type found, exploration complete");
                break;
            }

            std::list<FrontierCell> frontierA_cell_list;
            prepFrontierACell(frontierA_list, frontierA_cell_list);

            genInfoVec(frontierA_cell_list, frontierB_cell_list);
            frontierB_flag_ = (frontierB_cell_list.size() >= 5);

            // **************************************
            // ****** Test Area *********************
            //***************************************

            information_gain_vec_ = std::vector<double>(size_x_ * size_y_);

            visualize_info_gain(frontierA_cell_list);

            // ************************************

            ROS_INFO("Time flag proc frontier cell");

            if (flag_ugv_ && !ugv_flag_odom_){
                ROS_ERROR("UGV's odom not received! Didn't check UAV.");
                ros::Duration(1.0).sleep();
                continue;
            }

            if (flag_uav_ && !uav_flag_odom_){
                ROS_ERROR("UAV's odom not recieved!");
                ros::Duration(1.0).sleep();
                continue;
            }

            if (!flag_uav_){
                uav_pos = Eigen::VectorXd::Zero(4);
            }

            if (!flag_ugv_)
            {
		        ugv_pos = Eigen::VectorXd::Zero(6);
            }// TODO: Push something here

            start_pose_air.header    = uav_odom_.header;
            start_pose_air.pose      = uav_odom_.pose.pose;
            start_pose_ground.header = ugv_odom_.header;
            start_pose_ground.pose   = ugv_odom_.pose.pose;
            geometry_msgs::Twist ugv_acc, uav_acc;

            path_search search(uav_pos, ugv_pos, this, &frontierA_cell_list, wx_min, wx_max, wy_min, wy_max, path_search_rho_, ground_range_); // what is this???
            ROS_INFO("Time flag built search object");

            if (flag_ugv_){

                std::list<FrontierCell> node_list;

                if (search.find_max_path(search.get_ugv_tree(), ugv_coef, node_list)) { // if ugv tree has info gain
                    ROS_WARN("Time flag UGV: Selected!");
                    ugv_acc_pub_.publish(ugv_acc);

                    // visualize selected nodes
                    unsigned int intensity_viz = 10;
                    pcl::PointCloud<pcl::PointXYZI> frontierA_node_viz;
                    for (auto it = node_list.begin(); it != node_list.end(); it++){
                        pcl::PointXYZI frontier_point_viz(intensity_viz);
                        intensity_viz += 20;
                        double dist, viz_wx, viz_wy;
                        (layered_costmap_->getCostmap())->mapToWorld(
                                int(it->frontier_cell_point.x), int(it->frontier_cell_point.y),viz_wx, viz_wy);
                        frontier_point_viz.x = viz_wx;
                        frontier_point_viz.y = viz_wy;
                        frontierA_node_viz.push_back(frontier_point_viz);

                        computeInfoGainAndScoreGround(*it, frontierA_cell_list);
                        ROS_ERROR_STREAM("New computed score" << it->score);

                        for (std::list<FrontierCell>::iterator j = frontierA_cell_list.begin(); j != frontierA_cell_list.end(); j++){
                            // if raytrace no obstacle
                            dist = sqrt(pow(it->frontier_cell_point.x - j->frontier_cell_point.x,2) + pow(it->frontier_cell_point.y - j->frontier_cell_point.y,2));

                            if (dist <= ground_range_ / (layered_costmap_->getCostmap())->getResolution()) {

                                if(raytraceFree(int(it->frontier_cell_point.x), int(it->frontier_cell_point.y), int(j->frontier_cell_point.x), int(j->frontier_cell_point.y))){


                                    (layered_costmap_->getCostmap())->mapToWorld(
                                            int(j->frontier_cell_point.x), int(j->frontier_cell_point.y),viz_wx, viz_wy);
                                    frontier_point_viz.x = viz_wx;
                                    frontier_point_viz.y = viz_wy;
                                    frontierA_node_viz.push_back(frontier_point_viz);

                                }

                            }

                        }

                    }

                    sensor_msgs::PointCloud2 frontierA_node_viz_output;
                    pcl::toROSMsg(frontierA_node_viz, frontierA_node_viz_output);
                    frontierA_node_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
                    frontierA_node_viz_output.header.stamp = ros::Time::now();
                    frontierA_node_pub_.publish(frontierA_node_viz_output);

                    traj_viz(ugv_coef,1,search.get_plan_time_start(), true);

                    ugv_mat.header.frame_id = layered_costmap_->getGlobalFrameID();

                    Eigen::MatrixXd temp = ugv_coef;
                    ugv_coef.resize(ugv_coef.rows(),ugv_coef.cols()+2);
                    ugv_coef.block(0,0,ugv_coef.rows(),ugv_coef.cols()-2)=temp;
                    ugv_coef(0,ugv_coef.cols()-2)=ugv_pos(4);
                    ugv_coef(0,ugv_coef.cols()-1)=ugv_pos(5);
                    //std::cout<<ugv_coef.cols()<<"\n";

                    // push matrix
                    if (ugv_mat.array.layout.dim.size() != 2)
                        ugv_mat.array.layout.dim.resize(2);
                    ugv_mat.array.layout.dim[0].stride = ugv_coef.rows() * ugv_coef.cols();
                    ugv_mat.array.layout.dim[0].size = ugv_coef.rows();
                    ugv_mat.array.layout.dim[1].stride = ugv_coef.cols();
                    ugv_mat.array.layout.dim[1].size = ugv_coef.cols();
                    if ((int)ugv_mat.array.data.size() != ugv_coef.size())
                        ugv_mat.array.data.resize(ugv_coef.size());
                    int ii = 0;
                    for (int i = 0; i < ugv_coef.rows(); ++i)
                        for (int j = 0; j < ugv_coef.cols(); ++j)
                            ugv_mat.array.data[ii++] = ugv_coef.coeff(i, j);

                    // push goal_point with empty header
                    ugv_goal_point.header.frame_id = "";

                } else {
                    ROS_WARN("Time flag UGV: No Info Gain! Searching from FronA ...");
                    // publish matrix with empty size
                    ugv_mat.header.frame_id = "failsafe";
                    // publish goal_point here
                    std::list<FrontierCell> goal_candidate_ground_list;
                    std::list<Frontier> frontierA_inflated_list = frontierSearch.searchFrontierAFrom(
                            ugv_odom_.pose.pose.position);
                    prepFrontierACell(frontierA_inflated_list, goal_candidate_ground_list, start_pose_ground,
                                      start_pose_air, true, false);
                    getNextGroundFrontier(start_pose_ground, next_frontier_ground, frontierA_cell_list,
                                          goal_candidate_ground_list);

                    ugv_goal_point.header.stamp = ros::Time::now();
                    ugv_goal_point.header.frame_id = layered_costmap_->getGlobalFrameID();
                    ugv_goal_point.point = next_frontier_ground.pose.position;
                    
                }

                ugv_mat.header.stamp=ros::Time(search.get_plan_time_start());

                ugv_mat_pub_.publish(ugv_mat);
                ugv_goal_point_pub_.publish(ugv_goal_point);
                ROS_WARN("Time flag aft pub");

            }

            bool is_send_uav_goal = true;
            if (flag_uav_)
            {
                std::list<FrontierCell> node_list;

                if (search.find_max_path(search.get_uav_tree(), uav_coef, node_list)) { // if uav tree has info gain
                    ROS_WARN("Time flag UAV: Selected!");
		    uav_fail_flag = false;

                } else {
                    ROS_WARN("Time flag UAV: No Info Gain! Searching from FronS");
                    
                    
                    //find uav's goal
                    prepFrontierACell(frontierA_list, frontierA_cell_list, start_pose_ground, start_pose_air, false, true);
                    prepFrontierBCell(frontierB_cell_list, start_pose_air);

                    std::list<FrontierCell> goal_candidate_air_list;

                    std::list<FrontierCell>::iterator frontier_cell_it;
                    for (frontier_cell_it = frontierA_cell_list.begin();
                         frontier_cell_it != frontierA_cell_list.end(); frontier_cell_it++) {
                        goal_candidate_air_list.push_back(*frontier_cell_it);
                    }
                    for (frontier_cell_it = frontierB_cell_list.begin();
                         frontier_cell_it != frontierB_cell_list.end(); frontier_cell_it++) {
                        goal_candidate_air_list.push_back(*frontier_cell_it);
                    }
                    getNextAirFrontier(start_pose_air, next_frontier_air, frontierA_cell_list, frontierB_cell_list,
                                       goal_candidate_air_list, frontierB_cell_list.size() >= 5);
                    if(uav_fail_flag && (next_frontier_air.pose.position.x-prev_front_x)*(next_frontier_air.pose.position.x-prev_front_x)+(next_frontier_air.pose.position.y-prev_front_y)*(next_frontier_air.pose.position.y-prev_front_y)<0.25) is_send_uav_goal = false;
                    
                    TrajectoryGeneratorWaypoint fail;
                    Eigen::MatrixXd Path = Eigen::MatrixXd::Zero(2,3);
                    Eigen::MatrixXd Vel = Eigen::MatrixXd::Zero(2,3);
                    Eigen::MatrixXd Acc = Eigen::MatrixXd::Zero(2,3);
                    Eigen::VectorXd Time = Eigen::VectorXd::Zero(1);
                    
                    Path(0,0) = uav_pos(0);
                    Path(0,1) = uav_pos(2);
                    Path(1,0)= next_frontier_air.pose.position.x;
                    Path(1,1)= next_frontier_air.pose.position.y;
                    
                    Vel(0,0) = uav_pos(1);
                    Vel(0,1) = uav_pos(3);
                    Vel(1,0) = 0;//uav_pos(1);
                    Vel(1,1) = 0;//uav_pos(3);
                    
                    Time(0) = ((Path.row(0)-Path.row(1)).norm())/(Vel.row(0).norm());//
                    
                    uav_coef = fail.PolyQPGeneration(Path,Vel,Acc,Time);
                    Eigen::MatrixXd temp=uav_coef;
                    uav_coef.resize(temp.rows(),temp.cols()+2);
                    uav_coef.block(0,0,temp.rows(),temp.cols())=temp;
                    uav_coef(0,uav_coef.cols()-1)=Time(0)+search.get_plan_time_start();

                    prev_front_x=next_frontier_air.pose.position.x;
                    prev_front_y=next_frontier_air.pose.position.y;

                    if(!uav_fail_flag) uav_fail_flag=1;
                    
                }

                traj_viz(uav_coef,1,search.get_plan_time_start(), false);
                ROS_INFO("Time flag before pub UAV");

                if (uav_mat.array.layout.dim.size() != 2)
                    uav_mat.array.layout.dim.resize(2);
                uav_mat.array.layout.dim[0].stride = uav_coef.rows() * uav_coef.cols();
                uav_mat.array.layout.dim[0].size = uav_coef.rows();
                uav_mat.array.layout.dim[1].stride = uav_coef.cols();
                uav_mat.array.layout.dim[1].size = uav_coef.cols();
                if ((int)uav_mat.array.data.size() != uav_coef.size())
                    uav_mat.array.data.resize(uav_coef.size());
                int ii = 0;
                for (int i = 0; i < uav_coef.rows(); ++i)
                    for (int j = 0; j < uav_coef.cols(); ++j)
                        uav_mat.array.data[ii++] = uav_coef.coeff(i, j);

                uav_mat.header.stamp=ros::Time(search.get_plan_time_start());
                std::cout<<"plan time:"<<search.get_plan_time_start()<<std::endl;
                std::cout<<"header time:"<<uav_mat.header.stamp.toSec()<<std::endl;

		if (is_send_uav_goal)
                    uav_mat_pub_.publish(uav_mat);
		else
		    ROS_WARN("** UAV failsafe goal same as last ***");
                ROS_INFO("Time flag after pub UAV");

            }





            //visulize goal paths
            pcl::PointCloud<pcl::PointXYZI> ugv_tree_cloud_viz;
            visualization_msgs::Marker tree_marker;
            tree_marker.header.frame_id = layered_costmap_->getGlobalFrameID();
            tree_marker.header.stamp = ros::Time::now();
            tree_marker.action = visualization_msgs::Marker::ADD;
            tree_marker.pose.orientation.w = 1.0;
            tree_marker.id = 0;
            tree_marker.type = visualization_msgs::Marker::LINE_LIST;
            tree_marker.scale.x = 0.005;
            tree_marker.color.r = 1.0;
            tree_marker.color.a = 0.3;
            ugv_tree_cloud_viz = search.viz_tree(tree_marker); // visualize search tree
            tree_edge_viz_pub_.publish(tree_marker);

            sensor_msgs::PointCloud2 tree_viz_output;
            pcl::toROSMsg(ugv_tree_cloud_viz, tree_viz_output);
            tree_viz_output.header.frame_id = "/robot_1/map";
            tree_viz_output.header.stamp = ros::Time::now();
            ugv_tree_viz_pub_.publish(tree_viz_output);

            //publish trajectories
            traj_candidates_pub_.publish(tree_traj_marker_);
            std::cout << "point size: " << selected_marker_.points.size() << std::endl;
            selected_traj_pub_.publish(selected_marker_);
            tree_traj_marker_.points.clear();
            selected_marker_.points.clear();


            //push goal candidates into list, should change to motion primitive & LQR later
            /*
            {
                FrontierCell temp_goal_ground, temp_goal_air;
                unsigned int mx, my, index;
                double wx, wy;

                for (int i = 0; i < 12; i++) {
                    wx = start_pose_ground.pose.position.x + 1.0 * sin(i * 3.1416 / 6);
                    wy = start_pose_ground.pose.position.y + 1.0 * cos(i * 3.1416 / 6);
                    if (wx < 18.5 && wx > -0.5 && wy < 18.5 && wy > -0.5){// make sure goal is in boundary
                        (layered_costmap_->getCostmap())->worldToMap(wx, wy, mx, my);
                        if (layered_costmap_->getCostmap()->getCost(mx,my) == FREE_SPACE) {// make sure goal is FREE
                            temp_goal_ground.frontier_cell_point.x = mx;
                            temp_goal_ground.frontier_cell_point.y = my;
                            temp_goal_ground.traverse_cost_ground = 1.0;
                            goal_candidate_ground_list.push_back(temp_goal_ground);
                        }
                    }

                    wx = start_pose_air.pose.position.x + 1.0 * sin(i * 3.1416 / 6);
                    wy = start_pose_air.pose.position.y + 1.0 * cos(i * 3.1416 / 6);
                    if (wx < 18.5 && wx > -0.5 && wy < 18.5 && wy > -0.5) {
                        (layered_costmap_->getCostmap())->worldToMap(wx, wy, mx, my);
                        temp_goal_air.frontier_cell_point.x = mx;
                        temp_goal_air.frontier_cell_point.y = my;
                        temp_goal_air.traverse_cost_air = 1.0;
                        goal_candidate_air_list.push_back(temp_goal_air);
                    }
                }
            }
            */

            // get frontier goal for ugv and uav
            /*
            if (!getNextGroundFrontier(start_pose_ground, next_frontier_ground, frontierA_cell_list,
                                       goal_candidate_ground_list)) {
                // if no info gain with current goal candidates, change them to frontierA cells in FREE Space (inflation considered)
                goal_candidate_ground_list.clear();
                std::list<Frontier> frontierA_inflated_list = frontierSearch.searchFrontierAFrom(
                        start_pose_ground.pose.position);
                prepFrontierACell(frontierA_inflated_list, goal_candidate_ground_list, start_pose_ground,
                                  start_pose_air, true, false);
                getNextGroundFrontier(start_pose_ground, next_frontier_ground, frontierA_cell_list,
                                      goal_candidate_ground_list);
            }
            if (!getNextAirFrontier(start_pose_air, next_frontier_air, frontierA_cell_list, frontierB_cell_list,
                                    goal_candidate_air_list, frontierB_cell_list.size() >= 5)) {
                prepFrontierACell(frontierA_list, frontierA_cell_list, start_pose_ground, start_pose_air, false, true);
                prepFrontierBCell(frontierB_cell_list, start_pose_air);

                goal_candidate_air_list.clear();
                std::list<FrontierCell>::iterator frontier_cell_it;
                for (frontier_cell_it = frontierA_cell_list.begin();
                     frontier_cell_it != frontierA_cell_list.end(); frontier_cell_it++) {
                    goal_candidate_air_list.push_back(*frontier_cell_it);
                }
                for (frontier_cell_it = frontierB_cell_list.begin();
                     frontier_cell_it != frontierB_cell_list.end(); frontier_cell_it++) {
                    goal_candidate_air_list.push_back(*frontier_cell_it);
                }
                getNextAirFrontier(start_pose_air, next_frontier_air, frontierA_cell_list, frontierB_cell_list,
                                   goal_candidate_air_list, frontierB_cell_list.size() >= 5);
            }
             */

            // visualize frontier cells

            ROS_INFO("flag 3");
            visualize(frontierA_cell_list, frontierB_cell_list, next_frontier_ground, next_frontier_air);
            while (ros::Time::now().toSec() - loop_start_time < 3.0){
		ros::spinOnce();
		r.sleep();
	    }
		
	    //while((ugv_odom_.twist.twist.linear.x)*(ugv_odom_.twist.twist.linear.x)+(ugv_odom_.twist.twist.linear.y)*(ugv_odom_.twist.twist.linear.y)+(ugv_odom_.twist.twist.angular.z)*(ugv_odom_.twist.twist.angular.z)<0.1){
	//	ros::spinOnce();
	//	r.sleep();
	//	cnt++;
	//	if(cnt>25)break;
		
	//}
	std::cout<<"norm:"<<(ugv_odom_.twist.twist.linear.x)*(ugv_odom_.twist.twist.linear.x)+(ugv_odom_.twist.twist.linear.y)*(ugv_odom_.twist.twist.linear.y)+(ugv_odom_.twist.twist.angular.z)*(ugv_odom_.twist.twist.angular.z)<<std::endl;


        }

        ROS_ERROR("out of while");

        return true;

    }
    
    void AirGroundSimExploreLayer::getCandidateList(std::list<FrontierCell> &goal_candidate_ground_list, std::list<FrontierCell> &goal_candidate_air_list, double d_omega, double time_step, geometry_msgs::PoseStamped &start_pose_ground, geometry_msgs::PoseStamped &start_pose_air){
        FrontierCell temp_goal_ground, temp_goal_air;
        unsigned int mx, my;
        double wx, wy;
                    for (int i = 0; i < 12; i++) {
                        wx = start_pose_ground.pose.position.x + 1.0 * sin(i * 3.1416 / 6);
                        wy = start_pose_ground.pose.position.y + 1.0 * cos(i * 3.1416 / 6);
                        if (wx < 18.5 && wx > -0.5 && wy < 18.5 && wy > -0.5){// make sure goal is in boundary
                            (layered_costmap_->getCostmap())->worldToMap(wx, wy, mx, my);
                            if (layered_costmap_->getCostmap()->getCost(mx,my) == FREE_SPACE) {// make sure goal is FREE
                                temp_goal_ground.frontier_cell_point.x = mx;
                                temp_goal_ground.frontier_cell_point.y = my;
                                temp_goal_ground.traverse_cost_ground = 1.0;
                                goal_candidate_ground_list.push_back(temp_goal_ground);
                            }
                        }
        
                        wx = start_pose_air.pose.position.x + 1.0 * sin(i * 3.1416 / 6);
                        wy = start_pose_air.pose.position.y + 1.0 * cos(i * 3.1416 / 6);
                        if (wx < 18.5 && wx > -0.5 && wy < 18.5 && wy > -0.5) {
                            (layered_costmap_->getCostmap())->worldToMap(wx, wy, mx, my);
                            temp_goal_air.frontier_cell_point.x = mx;
                            temp_goal_air.frontier_cell_point.y = my;
                            temp_goal_air.traverse_cost_air = 1.0;
                            goal_candidate_air_list.push_back(temp_goal_air);
                        }
                    }
        
    }

    bool AirGroundSimExploreLayer::computeInfoGainAndScoreGround(FrontierCell &goal_candidate,
                                                         std::list<FrontierCell> &frontierA_cell_list) {
        goal_candidate.information_gain = 0; // clear leftover info gain

        double dist;
        unsigned int vis_fronA_cnt = 0; // visible frontier cell A count

        for (std::list<FrontierCell>::iterator j = frontierA_cell_list.begin(); j != frontierA_cell_list.end(); j++){
            // if raytrace no obstacle
            dist = sqrt(pow(goal_candidate.frontier_cell_point.x - j->frontier_cell_point.x,2)
                   + pow(goal_candidate.frontier_cell_point.y - j->frontier_cell_point.y,2));

            if (dist <= ground_range_ / (layered_costmap_->getCostmap())->getResolution()) {

                if(raytraceFree(int(goal_candidate.frontier_cell_point.x), int(goal_candidate.frontier_cell_point.y),
                                int(j->frontier_cell_point.x), int(j->frontier_cell_point.y))){

                    (goal_candidate.information_gain) += j->utility * log10(j->frontier_size) / dist;
                    vis_fronA_cnt++;
                }
            }
        }
        //if (vis_fronA_cnt != 0)
            //ROS_INFO("Visible frontier A: %d", vis_fronA_cnt);
        if (vis_fronA_cnt <= 5){ // reject goal candidate if can see less than 5 frontier cells
            goal_candidate.information_gain = 0;
            goal_candidate.score = 0.0;
            return false;
        }

        goal_candidate.score = pow(goal_candidate.information_gain, score_information_gain_coeff_);
				//   / pow(goal_candidate.traverse_cost_ground, score_traverse_cost_coeff_);


        return true;
    }

    bool AirGroundSimExploreLayer::getNextGroundFrontier(geometry_msgs::PoseStamped start_pose_ground,
                                                         geometry_msgs::PoseStamped &next_frontier_ground,
                                                         std::list<FrontierCell> &frontierA_cell_list,
                                                         std::list<FrontierCell> &goal_candidate_list)
    {
        // create placeholder for selected frontier cell
        frontier_exploration::FrontierCell selected;
        selected.score = 0.0;

        // compute information gain and score, select the highest
        for (std::list<FrontierCell>::iterator i = goal_candidate_list.begin(); i != goal_candidate_list.end(); i++){
            computeInfoGainAndScoreGround(*i, frontierA_cell_list);
            if (i->score > selected.score)
                selected = *i;
        }

        if (selected.score == 0.0){
            ROS_WARN("No info gain. Searching from frontierA cells.");
            return false;
        }

        // visualize visible frontierA
        /*
        pcl::PointCloud<pcl::PointXYZI> frontierA_visible_viz;
        pcl::PointXYZI frontier_point_viz(80);
        double viz_wx, viz_wy;
        for (std::list<FrontierCell>::iterator j = frontierA_cell_list.begin(); j != frontierA_cell_list.end(); j++){
            if (raytraceFree(int(selected.frontier_cell_point.x), int(selected.frontier_cell_point.y),
                             int(j->frontier_cell_point.x), int(j->frontier_cell_point.y))){
                (layered_costmap_->getCostmap())->mapToWorld(
                        (unsigned int) j->frontier_cell_point.x,
                        (unsigned int) j->frontier_cell_point.y,
                        viz_wx, viz_wy);
                frontier_point_viz.x = viz_wx;
                frontier_point_viz.y = viz_wy;
                frontierA_visible_viz.push_back(frontier_point_viz);
            }
        }
        (layered_costmap_->getCostmap())->mapToWorld(
                (unsigned int) selected.frontier_cell_point.x,
                (unsigned int) selected.frontier_cell_point.y,
                viz_wx, viz_wy);
        frontier_point_viz.x = viz_wx;
        frontier_point_viz.y = viz_wy;
        frontier_point_viz.intensity = 80;
        frontierA_visible_viz.push_back(frontier_point_viz);
        sensor_msgs::PointCloud2 frontierA_viz_output;
        pcl::toROSMsg(frontierA_visible_viz, frontierA_viz_output);
        frontierA_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontierA_viz_output.header.stamp = ros::Time::now();
        frontierA_visible_pub_.publish(frontierA_viz_output);
         */

        //set goal pose to next frontier
        next_frontier_ground.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier_ground.header.stamp = ros::Time::now();
        // get world coords from frontier_cell_point which is map coords
        double wx,wy;
        (layered_costmap_->getCostmap())->mapToWorld((unsigned int)selected.frontier_cell_point.x,
                                                     (unsigned int)selected.frontier_cell_point.y, wx, wy);

        next_frontier_ground.pose.position.x = wx;
        next_frontier_ground.pose.position.y = wy;
        next_frontier_ground.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose_ground.pose.position, next_frontier_ground.pose.position) );

        // perform utility discount
        BOOST_FOREACH(frontier_exploration::FrontierCell frontier_cell, frontierA_cell_list){
            if(raytraceFree((int)frontier_cell.frontier_cell_point.x, (int)frontier_cell.frontier_cell_point.y,
                            (int)selected.frontier_cell_point.x,      (int)selected.frontier_cell_point.y)){
                if (pow(frontier_cell.frontier_cell_point.x-selected.frontier_cell_point.x,2)// if distance is shorter than range
                    + pow(frontier_cell.frontier_cell_point.y-selected.frontier_cell_point.y,2)
                        <= pow(ground_range_,2)/layered_costmap_->getCostmap()->getResolution())
                {
                    frontier_cell.utility *= utility_discount_;
                }
            }
        }
        return true;
    }

    bool AirGroundSimExploreLayer::computeInfoGainAndScoreAir(FrontierCell &goal_candidate){

        goal_candidate.information_gain = 0;

        unsigned int left_mx, right_mx, top_my, bottom_my;

        unsigned int iter_x, iter_y;

        left_mx = (unsigned int)(goal_candidate.frontier_cell_point.x - 0.6/(layered_costmap_->getCostmap())->getResolution());
        right_mx = (unsigned int)(goal_candidate.frontier_cell_point.x + 0.6/(layered_costmap_->getCostmap())->getResolution());
        bottom_my = (unsigned int)(goal_candidate.frontier_cell_point.y - 0.6/(layered_costmap_->getCostmap())->getResolution());
        top_my = (unsigned int)(goal_candidate.frontier_cell_point.y + 0.6/(layered_costmap_->getCostmap())->getResolution());
        left_mx = left_mx > 0 ? left_mx : 0;
        right_mx = right_mx < size_x_ ? right_mx : size_x_ - 1;
        bottom_my = bottom_my > 0 ? bottom_my : 0;
        top_my = top_my < size_y_ ? top_my : size_y_ - 1;

        goal_candidate.information_gain = 0;

        unsigned int visible_frontier_cnt = 0, visible_frontierB_cnt = 0, idx;

        for (iter_y = bottom_my; iter_y <= top_my; iter_y++){

            for (iter_x = left_mx; iter_x <= right_mx; iter_x++){

                idx = (layered_costmap_->getCostmap())->getIndex(iter_x, iter_y); // get charMap index

                if (charmap_no_inflation_[idx] == NO_INFORMATION){
                    goal_candidate.information_gain += info_weight_unknown_;
                }

                if (frontierA_info_vec_[idx] > 0.0){
                    goal_candidate.information_gain += frontierA_info_vec_[idx];
                    visible_frontier_cnt ++;
                }

                if (frontierB_info_vec_[idx] > 0.0){
                    goal_candidate.information_gain += frontierB_info_vec_[idx];
                    visible_frontier_cnt ++;
                    visible_frontierB_cnt ++;
                }
            }
        }
        if (visible_frontier_cnt <= 3) // discard goal candidate without frontier visibility
        {
            goal_candidate.information_gain = 0;
        }

        if (visible_frontierB_cnt == 0 && frontierB_flag_){ // discard goal candidate without frontierB visibility if present
            goal_candidate.information_gain = 0;
        }

        // compute score and select the cell with the highest score
        goal_candidate.score = pow(goal_candidate.information_gain, score_information_gain_coeff_)
                               // * pow(goal_candidate.frontier_size, score_frontier_size_coeff_)
                               / pow(goal_candidate.traverse_cost_air, score_traverse_cost_coeff_);

        return !(goal_candidate.score == 0);
    }

    bool AirGroundSimExploreLayer::computeInfoGainAndScoreAir(FrontierCell &goal_candidate,
                                                              std::vector<double> &frontierA_info_vec,
                                                              std::vector<double> &frontierB_info_vec, bool frontierB_flag) {
        goal_candidate.information_gain = 0;

        unsigned int left_mx, right_mx, top_my, bottom_my;

        unsigned int iter_x, iter_y;

        left_mx = (unsigned int)(goal_candidate.frontier_cell_point.x - 0.6/(layered_costmap_->getCostmap())->getResolution());
        right_mx = (unsigned int)(goal_candidate.frontier_cell_point.x + 0.6/(layered_costmap_->getCostmap())->getResolution());
        bottom_my = (unsigned int)(goal_candidate.frontier_cell_point.y - 0.6/(layered_costmap_->getCostmap())->getResolution());
        top_my = (unsigned int)(goal_candidate.frontier_cell_point.y + 0.6/(layered_costmap_->getCostmap())->getResolution());
        left_mx = left_mx > 0 ? left_mx : 0;
        right_mx = right_mx < size_x_ ? right_mx : size_x_ - 1;
        bottom_my = bottom_my > 0 ? bottom_my : 0;
        top_my = top_my < size_y_ ? top_my : size_y_ - 1;

        goal_candidate.information_gain = 0;

        unsigned int visible_frontier_cnt = 0, visible_frontierB_cnt = 0, idx;

        for (iter_y = bottom_my; iter_y <= top_my; iter_y++){

            for (iter_x = left_mx; iter_x <= right_mx; iter_x++){

                idx = (layered_costmap_->getCostmap())->getIndex(iter_x, iter_y); // get charMap index

                if (charmap_no_inflation_[idx] == NO_INFORMATION){
                    goal_candidate.information_gain += info_weight_unknown_;
                }

                if (frontierA_info_vec[idx] > 0.0){
                    goal_candidate.information_gain += frontierA_info_vec[idx];
                    visible_frontier_cnt ++;
                }

                if (frontierB_info_vec[idx] > 0.0){
                    goal_candidate.information_gain += frontierB_info_vec[idx];
                    visible_frontier_cnt ++;
                    visible_frontierB_cnt ++;
                }
            }
        }
        if (visible_frontier_cnt <= 3) // discard goal candidate without frontier visibility
        {
            goal_candidate.information_gain = 0;
        }

        if (visible_frontierB_cnt == 0 && frontierB_flag){ // discard goal candidate without frontierB visibility if present
            goal_candidate.information_gain = 0;
        }

        // compute score and select the cell with the highest score
        goal_candidate.score = pow(goal_candidate.information_gain, score_information_gain_coeff_)
                               // * pow(goal_candidate.frontier_size, score_frontier_size_coeff_)
                               / pow(goal_candidate.traverse_cost_air, score_traverse_cost_coeff_);

    }

    bool AirGroundSimExploreLayer::getNextAirFrontier(geometry_msgs::PoseStamped start_pose_air,
                                                      geometry_msgs::PoseStamped &next_frontier_air,
                                                      std::list<FrontierCell> &frontierA_cell_list,
                                                      std::list<FrontierCell> &frontierB_cell_list,
                                                      std::list<FrontierCell> &goal_candidate_list, bool frontierB_flag)
    {
        // create placeholder for selected frontier cell
        frontier_exploration::FrontierCell selected;
        selected.score = 0.0;

        // Above is initialization. For each UAV, should do following operations.

        // compute info gain for each frontier cell
        // unsigned int debug_idx = 0;
        BOOST_FOREACH(frontier_exploration::FrontierCell goal_candidate, goal_candidate_list){ // changed here
            computeInfoGainAndScoreAir(goal_candidate);
            if (goal_candidate.score > selected.score)
                selected = goal_candidate;
        }

        if (selected.score == 0.0){
            ROS_WARN("AIR: No Info Gain. Searching from frontier cells.");
            return false;
        }

        //set goal pose to next frontier
        next_frontier_air.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier_air.header.stamp = ros::Time::now();
        // get world coords from frontier_cell_point which is map coords
        double selected_wx, selected_wy;
        (layered_costmap_->getCostmap())->mapToWorld((unsigned int)selected.frontier_cell_point.x,
                                                     (unsigned int)selected.frontier_cell_point.y,
                                                     selected_wx, selected_wy);

        next_frontier_air.pose.position.x = selected_wx;
        next_frontier_air.pose.position.y = selected_wy;
        next_frontier_air.pose.orientation = tf::createQuaternionMsgFromYaw( yawOfVector(start_pose_air.pose.position, next_frontier_air.pose.position) );

        // perform utility discount. Need re-write No use at present. For future cooperation between multiple UAVs.
        /*
        unsigned int iter_x, iter_y;
        // compute map_coords for UAV and fov_boundary, don't know if +- 1.5 works
        (layered_costmap_->getCostmap())->worldToMap(selected_wx - 1.5, selected_wy - 1.5,
                                                     left_mx, bottom_my);
        (layered_costmap_->getCostmap())->worldToMap(selected_wx + 1.5, selected_wy + 1.5,
                                                     right_mx, top_my);

        for (iter_y = bottom_my; iter_y <= top_my; iter_y++){
            for (iter_x = left_mx; iter_x <= right_mx; iter_x++){
                temp_idx = (layered_costmap_->getCostmap())->getIndex(iter_x, iter_y);
                information_gain_vector[temp_idx] *= utility_discount_;

            }
        }*/

        return true;
    }

    bool AirGroundSimExploreLayer::transformPose(geometry_msgs::PoseStamped& pose)
    {
        if(pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
            //error out if no transform available
            if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
                ROS_ERROR_STREAM("Transform pose: Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< pose.header.frame_id);
                return false;
            }
            geometry_msgs::PoseStamped temp_pose = pose;
            tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(), temp_pose, pose);
        }

        return true;
    }

    bool AirGroundSimExploreLayer::transformOdom(nav_msgs::Odometry &odom, std::string twist_frame) {

        if (odom.header.frame_id != layered_costmap_->getGlobalFrameID()){


            

            geometry_msgs::PoseStamped origin_pose, out_pose;
            origin_pose.header = odom.header;
            origin_pose.pose = odom.pose.pose;

            geometry_msgs::Vector3Stamped origin_v_l, origin_v_a, out_v_l, out_v_a;

            origin_v_l.header = origin_v_a.header = odom.header;
            if (!twist_frame.empty() && flag_is_sim_){
		ROS_INFO_STREAM("UGV in Sim: set frame to " << twist_frame);
                origin_v_a.header.frame_id = twist_frame;
                origin_v_l.header.frame_id = twist_frame;
            }

            origin_v_a.vector = odom.twist.twist.angular;
            origin_v_l.vector = odom.twist.twist.linear;



            if (!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), odom.header.frame_id, odom.header.stamp, ros::Duration(10))){
                ROS_ERROR_STREAM("Transform Odom: Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< odom.header.frame_id);
                return false;
            }

            while (true){
            	try{
            		tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(), origin_pose, out_pose);
		            tf_listener_.transformVector(layered_costmap_->getGlobalFrameID(), origin_v_a, out_v_a);
		            tf_listener_.transformVector(layered_costmap_->getGlobalFrameID(), origin_v_l, out_v_l);

		            break;
            	}catch(std::exception &e){
            		std::cout << e.what() << std::endl;
                    ros::Time stamped_postponed(odom.header.stamp.toSec() + 0.1);
                    tf_listener_.waitForTransform(global_frame_, odom.header.frame_id, stamped_postponed, ros::Duration(5));
            	}
            }
           

            odom.header = out_pose.header;
            odom.pose.pose = out_pose.pose;
            odom.twist.twist.linear = out_v_l.vector;
            odom.twist.twist.angular = out_v_a.vector;
        }

        return true;
    }

    void AirGroundSimExploreLayer::prepFrontierACell(std::list<Frontier> &frontierA_list, std::list<FrontierCell> &frontierA_cell_list)
    {
        frontierA_cell_list.clear();

        BOOST_FOREACH(frontier_exploration::Frontier frontier, frontierA_list) {

                        BOOST_FOREACH(geometry_msgs::Point frontier_cell_point, frontier.frontier_cells) {

                                        frontier_exploration::FrontierCell temp;
                                        temp.frontier_cell_point = frontier_cell_point;
                                        temp.frontier_size = frontier.size;
                                        temp.utility = 1.0;

                                        frontierA_cell_list.push_back(temp);
                                    }
                    }
    }


    void AirGroundSimExploreLayer::prepFrontierACell(std::list<Frontier> &frontierA_list, std::list<FrontierCell> &frontierA_cell_list,
                                                     geometry_msgs::PoseStamped start_pose_ground, geometry_msgs::PoseStamped start_pose_air,
                                                     bool compute_ground_cost, bool compute_air_cost)
    {
        frontierA_cell_list.clear();

        ros::NodeHandle n;
        ros::ServiceClient make_plan_client = n.serviceClient<nav_msgs::GetPlan>("/move_base_node/make_plan");
        nav_msgs::GetPlan srv;
        if(!make_plan_client.waitForExistence()){
            ROS_ERROR("Timeout waiting for service /move_base_node/make_plan");
        }

        geometry_msgs::PoseStamped start, goal;

        BOOST_FOREACH(frontier_exploration::Frontier frontier, frontierA_list){
                        double wx,wy, air_distance, ground_distance = 0.0, temp_length;
                        //frontier_exploration::AstarPoint startPoint, endPoint;
                        //int temp_cost;
                        BOOST_FOREACH(geometry_msgs::Point frontier_cell_point, frontier.frontier_cells){

                                        frontier_exploration::FrontierCell temp;
                                        temp.frontier_cell_point = frontier_cell_point;
                                        temp.frontier_size = frontier.size;
                                        temp.utility = 1.0;
                                        temp.score = 0.0;

                                        // get world coords from frontier_cell_point which is map coords

                                        (layered_costmap_->getCostmap())->mapToWorld((unsigned int)temp.frontier_cell_point.x,
                                                                                     (unsigned int)temp.frontier_cell_point.y, wx, wy);

                                        if (compute_air_cost){
                                            air_distance = sqrt((wx-start_pose_air.pose.position.x)*(wx-start_pose_air.pose.position.x)
                                                                + (wy-start_pose_air.pose.position.y) * (wy-start_pose_air.pose.position.y));
                                            temp.traverse_cost_air = air_distance;
                                        }

                                        if (compute_ground_cost){
                                            // compute and add traverse_cost
                                            start = start_pose_ground;
                                            goal.header = start.header;
                                            goal.pose.orientation = start.pose.orientation;
                                            goal.pose.position.x = wx;
                                            goal.pose.position.y = wy;
                                            srv.request.start = start;
                                            srv.request.goal  = goal;

                                            if (make_plan_client.call(srv) && srv.response.plan.poses.size()) {
                                                std::vector<geometry_msgs::PoseStamped>::iterator i;
                                                for (i = srv.response.plan.poses.begin(); (i + 1) != srv.response.plan.poses.end(); i++) {
                                                    temp_length = sqrt(
                                                            ((i + 1)->pose.position.x - i->pose.position.x)
                                                            * ((i + 1)->pose.position.x - i->pose.position.x)
                                                            + ((i + 1)->pose.position.y - i->pose.position.y)
                                                              * ((i + 1)->pose.position.y - i->pose.position.y));
                                                    ground_distance += temp_length;
                                                }
                                                temp.traverse_cost_ground = ground_distance;


                                            } else {
                                                //ROS_WARN("Failed to call make plan service, setting to infinity");
                                                temp.traverse_cost_ground = std::numeric_limits<double>::infinity();
                                            }


                                            /* code below uses astar class. Abandoned for its low speed.
                                            unsigned int mx, my;
                                            layered_costmap_->getCostmap()->worldToMap(start_pose_ground.pose.position.x,
                                                                                       start_pose_ground.pose.position.x,mx,my);
                                            startPoint.x = mx;
                                            startPoint.y = my;
                                            endPoint.x = (int)frontier_cell_point.x;
                                            endPoint.y = (int)frontier_cell_point.y;
                                            temp_cost = astarPlanner.getCost(startPoint, endPoint, false);
                                            temp.traverse_cost_ground = (double)temp_cost;

                                            ROS_INFO_STREAM("Cost of frontier cell is " << temp_cost);
                                            */
                                        }



                                        frontierA_cell_list.push_back(temp);
                                    }
                    }
    }

    void AirGroundSimExploreLayer::prepFrontierBCell(std::list<FrontierCell> &frontierB_cell_list, geometry_msgs::PoseStamped start_pose_air) {
        std::list<FrontierCell>::iterator it;
        double wx = start_pose_air.pose.position.x, wy = start_pose_air.pose.position.y, world_distance;
        for (it = frontierB_cell_list.begin(); it != frontierB_cell_list.end(); it++){

            (layered_costmap_->getCostmap())->mapToWorld((unsigned int)it->frontier_cell_point.x,
                                                         (unsigned int)it->frontier_cell_point.y, wx, wy);
            // compute traverse cost

            world_distance = sqrt((wx-start_pose_air.pose.position.x)*(wx-start_pose_air.pose.position.x)
                                  + (wy-start_pose_air.pose.position.y) * (wy-start_pose_air.pose.position.y));
            it->traverse_cost_air = world_distance;
        }
    }

    void AirGroundSimExploreLayer::visualize(std::list<FrontierCell> &frontierA_cell_list, std::list<FrontierCell> &frontierB_cell_list,geometry_msgs::PoseStamped &next_frontier_ground, geometry_msgs::PoseStamped &next_frontier_air){

        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cell_viz;
        pcl::PointXYZI frontier_point_viz(50);//initialize with 50 intensi      ty
        pcl::PointXYZI frontier_point_viz_B(30);//initialize viz for frontierB with 30 intensity
        double viz_wx, viz_wy;
        //ROS_INFO_STREAM("Size of frontierA list: " << frontierA_cell_list.size());
        BOOST_FOREACH(FrontierCell frontier_cell, frontierA_cell_list) {
                        //load frontier into visualization poitncloud
                        (layered_costmap_->getCostmap())->mapToWorld(
                                (unsigned int) frontier_cell.frontier_cell_point.x,
                                (unsigned int) frontier_cell.frontier_cell_point.y,
                                viz_wx, viz_wy);
                        frontier_point_viz.x = viz_wx;
                        frontier_point_viz.y = viz_wy;
                        frontier_cell_viz.push_back(frontier_point_viz);
                    }
        //ROS_INFO_STREAM("Size of frontierB list: " << frontierB_cell_list.size());
        BOOST_FOREACH(FrontierCell frontier_cell, frontierB_cell_list) {
                        //load frontier into visualization poitncloud
                        (layered_costmap_->getCostmap())->mapToWorld(
                                (unsigned int) frontier_cell.frontier_cell_point.x,
                                (unsigned int) frontier_cell.frontier_cell_point.y,
                                viz_wx, viz_wy);
                        frontier_point_viz_B.x = viz_wx;
                        frontier_point_viz_B.y = viz_wy;
			frontier_point_viz_B.z = 1.0;
                        frontier_cell_viz.push_back(frontier_point_viz_B);
                    }
        // push next_frontier_air into viz list
        frontier_point_viz.x = (float) next_frontier_air.pose.position.x;
        frontier_point_viz.y = (float) next_frontier_air.pose.position.y;
        frontier_cell_viz.push_back(frontier_point_viz);
        frontier_cell_viz[frontier_cell_viz.size() - 1].intensity = 20;
        // push next_frontier_ground into viz list
        frontier_point_viz.x = (float) next_frontier_ground.pose.position.x;
        frontier_point_viz.y = (float) next_frontier_ground.pose.position.y;
        frontier_cell_viz.push_back(frontier_point_viz);
        frontier_cell_viz[frontier_cell_viz.size() - 1].intensity = 100;

        //publish visualization point cloud
        sensor_msgs::PointCloud2 frontier_cell_viz_output;
        pcl::toROSMsg(frontier_cell_viz, frontier_cell_viz_output);
        frontier_cell_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontier_cell_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub.publish(frontier_cell_viz_output);

        ROS_INFO("Time flag viz");
    }

    bool AirGroundSimExploreLayer::raytraceFree(int x0, int y0, int x1, int y1){

        int dx = x1 - x0;
        int dy = y1 - y0;

        unsigned int abs_dx = abs(dx); // abs delta x
        unsigned int abs_dy = abs(dy); // abs delta y

        unsigned int size_x = (layered_costmap_->getCostmap())->getSizeInCellsX();

        int offset_dx = (dx > 0 ? 1 : -1); // dx in map idx
        int offset_dy = (dy > 0 ? 1 : -1) * size_x; // dy in map idx

        unsigned int max_length = std::numeric_limits<unsigned int>::max();

        unsigned int offset = y0 * size_x + x0; // point0 idx

        double dist = hypot(dx, dy); // 2D distance
        double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

        if (abs_dx >= abs_dy)
        {
            int error_y = abs_dx / 2;
            return bresenham2D_raytraceFree(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        }else{
            int error_x = abs_dy / 2;
            return bresenham2D_raytraceFree(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
        }
    }

    bool AirGroundSimExploreLayer::raytraceFree3D(int x0, int y0, int x1, int y1){ // point0 should be initial point (frontierB_cell)

        int dx = x1 - x0;
        int dy = y1 - y0;

        unsigned int abs_dx = abs(dx); // abs delta x
        unsigned int abs_dy = abs(dy); // abs delta y

        unsigned int size_x = (layered_costmap_->getCostmap())->getSizeInCellsX();

        int offset_dx = (dx > 0 ? 1 : -1); // dx in map idx
        int offset_dy = (dy > 0 ? 1 : -1) * size_x; // dy in map idx

        unsigned int max_length = std::numeric_limits<unsigned int>::max();

        unsigned int offset = y0 * size_x + x0; // point0 idx

        double dist = hypot(dx, dy); // 2D distance
        double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

        if (abs_dx >= abs_dy)
        {
            int error_y = abs_dx / 2;
            return bresenham2D_raytraceFree3D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        }else{
            int error_x = abs_dy / 2;
            return bresenham2D_raytraceFree3D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
        }
    }

    bool AirGroundSimExploreLayer::bresenham2D_raytraceFree(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length){

        // abs_da > abs_db

        unsigned int end = std::min(max_length, abs_da);

        unsigned int mx, my;
        for (unsigned int i = 0; i < end; ++i)
        {
            (layered_costmap_->getCostmap())->indexToCells(offset, mx, my);
            if ((layered_costmap_->getCostmap())->getCost(mx, my)  != FREE_SPACE
               && (layered_costmap_->getCostmap())->getCost(mx, my) != NO_INFORMATION) 
            {
                return false;
            }
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }

            /*
            double viz_wx, viz_wy;
            pcl::PointXYZI frontier_point_viz(80);
            (layered_costmap_->getCostmap())->mapToWorld(
                    mx, my,viz_wx, viz_wy);
            frontier_point_viz.x = viz_wx;
            frontier_point_viz.y = viz_wy;
            frontierA_visible_viz.push_back(frontier_point_viz);
             */

        }

        if ((layered_costmap_->getCostmap())->getCost(mx, my)  != FREE_SPACE
            && (layered_costmap_->getCostmap())->getCost(mx, my) != NO_INFORMATION)// NEVER CHANGE THIS
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool AirGroundSimExploreLayer::bresenham2D_raytraceFree3D(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length){

        if (abs_da <= 3){
            return true;
        }

        // abs_da > abs_db
        unsigned int mx, my;

        double block_da = abs_da / uav_height_ * obstacle_height_;

        for (unsigned int i = 0; i < (int)block_da; ++i)
        {
            (layered_costmap_->getCostmap())->indexToCells(offset, mx, my);
            if ((layered_costmap_->getCostmap())->getCost(mx, my) != FREE_SPACE)
            {
                return false;
            }
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int)error_b >= abs_da)
            {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        (layered_costmap_->getCostmap())->indexToCells(offset, mx, my);
        if ((layered_costmap_->getCostmap())->getCost(mx, my) != FREE_SPACE)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    void AirGroundSimExploreLayer::genCharmapNoInflation() {

        unsigned int charmap_size = size_x_ * size_y_;
        bool static_0_flag = false, sensor_1_flag = false, sensor_0_flag = false;
        charmap_no_inflation_ = std::vector<unsigned char>(charmap_size);
        boost::shared_ptr<uav_obstacle_layer::UAVObstacleLayer> uav_obstacle_layer, ground_obstacle_layer, static_layer;
        //unsigned char *charmap_air, *charmap_ground, *charmap_static;
        std::vector<boost::shared_ptr<costmap_2d::Layer> > *plugins = layered_costmap_->getPlugins();
        for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin();
             pluginp != plugins->end(); ++pluginp) {

            boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;

            if (plugin->getName().find("sensor_0") != std::string::npos) {
                uav_obstacle_layer = boost::static_pointer_cast<uav_obstacle_layer::UAVObstacleLayer>(plugin);
                //inflated_visited = uav_obstacle_layer->inflated_visited_;
                sensor_0_flag = true;
            }

            if (plugin->getName().find("sensor_1") != std::string::npos) {
                ground_obstacle_layer = boost::static_pointer_cast<uav_obstacle_layer::UAVObstacleLayer>(
                        plugin);
                sensor_1_flag = true;
            }
            if (plugin->getName().find("static_0") != std::string::npos) {
                static_layer = boost::static_pointer_cast<uav_obstacle_layer::UAVObstacleLayer>(plugin);
                static_0_flag = true;
            }
        }

        // Merge all charmaps without inflation

        unsigned int mx, my;
        unsigned char cost_air, cost_ground, cost_static;
        for (int i = 0; i < size_x_ * size_y_; i++) {
            indexToCells(i, mx, my);


            if (sensor_0_flag)
                cost_air = uav_obstacle_layer->getCost(mx, my);
            else
                cost_air = NO_INFORMATION;

            if (sensor_1_flag)
                cost_ground = ground_obstacle_layer->getCost(mx, my);
            else
                cost_ground = NO_INFORMATION;

            if (static_0_flag)
                cost_static = static_layer->getCost(mx, my);
            else
                cost_static = NO_INFORMATION;

            if (cost_air != NO_INFORMATION)
                charmap_no_inflation_[i] = cost_air;
            else if (cost_ground == LETHAL_OBSTACLE || cost_static == LETHAL_OBSTACLE)
                charmap_no_inflation_[i] = LETHAL_OBSTACLE;
            else if (cost_ground == FREE_SPACE || cost_static == FREE_SPACE)
                charmap_no_inflation_[i] = FREE_SPACE;
            else
                charmap_no_inflation_[i] = NO_INFORMATION;

        }
    }

    void AirGroundSimExploreLayer::pubGridmapNoInflation() {

        grid_map_no_inflation_.info.width             = layered_costmap_->getCostmap()->getSizeInCellsX();
        grid_map_no_inflation_.info.height            = layered_costmap_->getCostmap()->getSizeInCellsY();

        unsigned char tmp_map_val;

        grid_map_no_inflation_.data.clear();

        for (unsigned int idx = 0; idx < size_x_ * size_y_; idx++){
            tmp_map_val = charmap_no_inflation_[idx];
            grid_map_no_inflation_.data.push_back(tmp_map_val);
        }

        gridmap_no_inflation_pub_.publish(grid_map_no_inflation_);
    }

    void AirGroundSimExploreLayer::genInfoVec(std::list<FrontierCell> &frontierA_cell_list, std::list<FrontierCell> &frontierB_cell_list) {
        frontierA_info_vec_ = std::vector<double>((size_x_ * size_y_), 0.0);
        frontierB_info_vec_ = std::vector<double>((size_x_ * size_y_), 0.0);
        unsigned int temp_idx;
        BOOST_FOREACH(frontier_exploration::FrontierCell frontierA_cell, frontierA_cell_list) {
                        temp_idx = (layered_costmap_->getCostmap())->getIndex(
                                (unsigned int) frontierA_cell.frontier_cell_point.x,
                                (unsigned int) frontierA_cell.frontier_cell_point.y);
                        frontierA_info_vec_[temp_idx] = info_weight_frontierA_ * frontierA_cell.utility;

                    }
        BOOST_FOREACH(frontier_exploration::FrontierCell frontierB_cell, frontierB_cell_list) {
                        temp_idx = (layered_costmap_->getCostmap())->getIndex(
                                (unsigned int) frontierB_cell.frontier_cell_point.x,
                                (unsigned int) frontierB_cell.frontier_cell_point.y);
                        frontierB_info_vec_[temp_idx] = info_weight_frontierB_ * frontierB_cell.utility;
                    }
    }

    void AirGroundSimExploreLayer::countProgress(){

        unsigned int explored_cnt = 0;
        for(unsigned int i = 0; i < charmap_no_inflation_.size(); i++){
            if (charmap_no_inflation_[i] != NO_INFORMATION)
                explored_cnt++;
        }
        if (!charmap_no_inflation_.empty())
            ROS_INFO_STREAM("Exploration progress: " << (double(explored_cnt) / charmap_no_inflation_.size()) * 100 << "%");
    }

    // ***************** Above are new functions ***************************

    bool AirGroundSimExploreLayer::updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res){

        ROS_WARN("Got update srv req!");

        return updateBoundaryPolygon(req.explore_boundary);

    }

    void AirGroundSimExploreLayer::reset(){

        //reset costmap_ char array to default values
        marked_ = false;
        configured_ = false;
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

    }

    bool AirGroundSimExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

        //clear existing boundary, if any
        polygon_.points.clear();

        //error if no transform available between polygon and costmap
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Update Boundary: Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
            return false;
        }

        //Transform all points of boundary polygon into costmap frame
        geometry_msgs::PointStamped in, out;
        in.header = polygon_stamped.header;
        BOOST_FOREACH(geometry_msgs::Point32 point32, polygon_stamped.polygon.points){
            in.point = costmap_2d::toPoint(point32);
            tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
            polygon_.points.push_back(costmap_2d::toPoint32(out.point));
        }

        //if empty boundary provided, set to whole map
        if(polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if(resize_to_boundary_){
            updateOrigin(0,0);

            //Find map size and origin by finding min/max points of polygon
            double min_x = std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.points){
                min_x = std::min(min_x,(double)point.x);
                min_y = std::min(min_y,(double)point.y);
                max_x = std::max(max_x,(double)point.x);
                max_y = std::max(max_y,(double)point.y);
            }

            //resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
            matchSize();
        }

        configured_ = true;
        marked_ = false;
        return true;
    }

    void AirGroundSimExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //update the whole costmap
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();
    }

    void AirGroundSimExploreLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        //check if layer is enabled and configured with a boundary
        if (!enabled_ || !configured_){ return; }

        //draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        //circular iterator
        for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){

            int x_1, y_1, x_2, y_2;
            worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);
        }
        //update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);


    }

    void AirGroundSimExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        if (!enabled_)
            return;

        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
                    master[it] = costmap_[it];
                }
                it++;
            }
        }
        marked_ = true;
    }

    bool AirGroundSimExploreLayer::blacklistPointService(frontier_exploration::BlacklistPoint::Request &req, frontier_exploration::BlacklistPoint::Response &res) {
        // Add point to blacklist
        blacklist_.push_back(req.point);
        ROS_WARN("Blacklist point added %f, %f", req.point.x, req.point.y);

        // Show point in blacklist topic
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "blacklist";
        marker.id = blacklist_.size();
        marker.action = visualization_msgs::Marker::ADD;

        marker.header.frame_id = global_frame_;
        marker.header.stamp = ros::Time::now();

        marker.pose.position = req.point;
        marker.pose.orientation.w = 1.0;

        // Scale is the diameter of the shape
        marker.scale.x = 2 * blacklist_radius_;
        marker.scale.y = 2 * blacklist_radius_;
        // Circle
        marker.scale.z = 0.05;

        marker.color.r = 1.0;
        marker.color.a = 0.6;

        blacklist_marker_pub_.publish(marker);

        // All is good :)
        return true;
    }

    bool AirGroundSimExploreLayer::clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
        // Clear the list
        blacklist_.clear();
        ROS_WARN("Blacklist cleared");

        // Delete all markers from visualization
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "blacklist";
        // The constant does not exist in ROS Indigo, although functionality is implemented. We use our own.
        marker.action = DELETEALL;

        // All is good :)
        return true;
    }
    
    void AirGroundSimExploreLayer::uav_odom_call_back(const nav_msgs::Odometry &uav_odom_origin){

        //ROS_WARN("receive uav odom");
        uav_odom_ = uav_odom_origin;
        transformOdom(uav_odom_,"");

        Eigen::VectorXd state = Eigen::VectorXd::Zero(4);
        state(0)=uav_odom_.pose.pose.position.x;
        state(1)=uav_odom_.twist.twist.linear.x;
        state(2)=uav_odom_.pose.pose.position.y;
        state(3)=uav_odom_.twist.twist.linear.y;
        uav_pos = state;

        uav_flag_odom_ = true;
    }
    
    void AirGroundSimExploreLayer::ugv_odom_call_back(const nav_msgs::Odometry ugv_odom_origin){

        //ROS_WARN("receive ugv odom");

        ugv_odom_ = ugv_odom_origin;
        transformOdom(ugv_odom_, "/robot_1/base_link");

        Eigen::Quaterniond ori(ugv_odom_.pose.pose.orientation.w,  ugv_odom_.pose.pose.orientation.x,
                        ugv_odom_.pose.pose.orientation.y,  ugv_odom_.pose.pose.orientation.z);
        Eigen::Matrix3d Rgi = ori.toRotationMatrix();
        Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
        state(0)=ugv_odom_.pose.pose.position.x;
        state(1)=ugv_odom_.twist.twist.linear.x;
        state(2)=ugv_odom_.pose.pose.position.y;

        //std::cout << "state: " << state(2) << ", y: " << ugv_odom_.pose.pose.position.y << std::endl;
        state(3)=ugv_odom_.twist.twist.linear.y;

        double phi = std::asin(Rgi(2,1));
        state(4)=atan2(-Rgi(0,1)/cos(phi),Rgi(1,1)/cos(phi));
        state(5)=ugv_odom_.twist.twist.angular.z;
        ugv_pos = state;

        ugv_flag_odom_ = true;
    }
}
