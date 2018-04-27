#ifndef PATH_SEARCH_H_
#define PATH_SEARCH_H_
#include <iostream>
#include <vector>
#include <list>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>
#include <duet_exploration/harmonic_duet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <geometry_msgs/PoseArray.h>



namespace duet_exploration{

    class search_tree{
    public:
        search_tree() : cost(-1),score(-1) {}
        search_tree(search_tree* parent,Eigen::VectorXd input,Eigen::VectorXd state,double time, double time_step,
                    std::vector<Eigen::VectorXd> &input_list, unsigned int n_layers, double info_gain, double cost, double score);

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
        double info_gain;
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
        root_node(Eigen::VectorXd state, double time_step, std::vector<Eigen::VectorXd> input_list, unsigned int n_layers)
                : search_tree(nullptr, Eigen::VectorXd::Zero(input_list[0].size()),state,0.0,time_step,input_list, n_layers, 0.0, 0.0, 0.0)
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
        path_search(Eigen::VectorXd uav_plan_pos, double plan_time_start);
        bool find_max_path();

        ~path_search(){
            delete uav_tree;
        }

        root_node* get_uav_tree(){
            return uav_tree;
        }

        double get_plan_time_start(){
            return plan_time_start;
        }

        pcl::PointCloud<pcl::PointXYZI>& viz_tree(visualization_msgs::Marker &tree_marker);

    private:
        root_node* uav_tree;
        double plan_time_start;

        ros::NodeHandle nh_;
        pcl::PointCloud<pcl::PointXYZI> uav_tree_cloud_viz_;
        ros::Publisher uav_tree_viz_pub_;
        ros::Publisher uav_tree_point_viz_pub_;
        ros::Publisher uav_waypts_pub_;
        void viz_tree_recursive(search_tree *tree, visualization_msgs::Marker &tree_marker);
    };
}


#endif
