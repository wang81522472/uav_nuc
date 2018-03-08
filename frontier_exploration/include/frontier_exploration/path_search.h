#ifndef PATH_SEARCH_H_
#define PATH_SEARCH_H_
#include <iostream>
#include <vector>
#include <list>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>
#include <frontier_exploration/air_ground_sim_explore_layer.h>
#include <frontier_exploration/trajectory_generator_waypoint.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <boost/heap/fibonacci_heap.hpp>



namespace frontier_exploration{

    class search_tree{
    public:
        search_tree() : cost(-1),score(-1) {}
        search_tree(search_tree* parent,Eigen::VectorXd input,Eigen::VectorXd state,double time, double time_step,
                    std::vector<Eigen::VectorXd> &input_list, unsigned int n_layers, double cost, double score, bool check_collision,
                    frontier_exploration::AirGroundSimExploreLayer* service, std::list<FrontierCell>* frontierA_cell_list);

        ~search_tree(){
            for(auto it= children.rbegin(); it != children.rend(); ++it){
                delete *it;
            }
        }

	

        Eigen::VectorXd get_input(){ return input;}
        Eigen::VectorXd get_state(){ return state;}
        double get_time(){ return time;}
        double get_cost(){ return cost;}
        double get_score()const{ return score;}

        search_tree* get_parent(){return parent;}

        std::vector<search_tree*>& get_children(){
            return this->children;
        }

        void set_max_leaf_node(search_tree* max_leaf)
	{
	    this->max_node=max_leaf;
	}

        search_tree* get_max_node()
	{
	    return this->max_node;
	}
        struct search_tree_node_compare
        {
            bool operator()(search_tree* const &e1, search_tree* const &e2) const
            {
                return e1->get_score()<e2->get_score();
            }
        };
        boost::heap::fibonacci_heap<search_tree*, boost::heap::compare<search_tree_node_compare>>* get_f_heap(){
            return &f_heap;
        }
        
        double get_time_step(){
            return time_step;
        }
    private:
        search_tree* parent;
        std::vector<search_tree*> children;
        Eigen::VectorXd input;
        Eigen::VectorXd state;
        double time;
        double cost;
        double score;
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        search_tree* max_node;
        boost::heap::fibonacci_heap<search_tree*, boost::heap::compare<search_tree_node_compare>> f_heap;
        double time_step;
    };

    class root_node : public search_tree{
    public:
        root_node():search_tree(){};
        root_node(Eigen::VectorXd state, double time_step, std::vector<Eigen::VectorXd> input_list, unsigned int n_layers, bool check_collision,
                  frontier_exploration::AirGroundSimExploreLayer* service, std::list<FrontierCell>* frontierA_cell_list)
                : search_tree(nullptr, Eigen::VectorXd::Zero(input_list[0].size()),state,0.0,time_step,input_list, n_layers, 0.0, 0.0, check_collision,service, frontierA_cell_list)
        {
            this->n_layers=n_layers;
            this->input_list=input_list;
            
        }

    private:
        unsigned int n_layers;
        std::vector<Eigen::VectorXd> input_list;

    };

    class path_search{
    public:
        path_search();
        path_search(Eigen::VectorXd uav_plan_pos,Eigen::VectorXd ugv_plan_pos, frontier_exploration::AirGroundSimExploreLayer* service,
                    std::list<FrontierCell>* frontierA_cell_list, double wx_min_param, double wx_max_param, double wy_min_param,
                    double wy_max_param, double rho_param, double ground_range);
        bool find_max_path(root_node* tree, Eigen::MatrixXd &coef, std::list<FrontierCell> &node_list);
        bool collision_free(search_tree* max_node);

        ~path_search(){
            delete uav_tree;
            delete ugv_tree;
        }

        root_node* get_uav_tree(){
            return uav_tree;
        }

        root_node* get_ugv_tree(){
            return ugv_tree;
        }

        double get_plan_time_start(){
            return plan_time_start;
        }

        pcl::PointCloud<pcl::PointXYZI>& viz_tree(visualization_msgs::Marker &tree_marker);

    private:
        root_node* uav_tree;
        root_node* ugv_tree;
        frontier_exploration::AirGroundSimExploreLayer* service_;
        std::list<FrontierCell>* frontierA_cell_list;
        double plan_time_start;

        ros::NodeHandle nh_;
        pcl::PointCloud<pcl::PointXYZI> uav_tree_cloud_viz_, ugv_tree_cloud_viz_;
        ros::Publisher uav_tree_viz_pub_,ugv_tree_viz_pub_, ugv_input_pub_;
        void viz_tree_recursive(search_tree *tree, bool is_ugv, visualization_msgs::Marker &tree_marker);
    };
}


#endif
