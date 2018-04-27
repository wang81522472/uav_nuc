#include <duet_exploration/path_search.h>
#include <boost/foreach.hpp>
#include <costmap_2d/cost_values.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>



extern duet_exploration::DuetExploration * explorer_;


#define layers 4

#define uav_max_vx 1
#define uav_max_vy 1
#define uav_z 2.0

#define x_in_state 0
#define vx_in_state 1
#define y_in_state 2
#define vy_in_state 3

double wx_min, wx_max, wy_min, wy_max;
#define tree_time_step 2

#define rho 1

#define uav_input_length 2
#define uav_input_step 0.1
#define uav_ninput 5

#define set_UAV_A 0,1,0,0, 0,0,0,0, 0,0,0,1, 0,0,0,0
#define set_UAV_B 0,0, 1,0, 0,0, 0,1


namespace duet_exploration {
    using namespace std;
    using namespace Eigen;


    inline int fac1(int k){
        return k<2 ? 1:k*fac1(k-1);
    }

    Eigen::MatrixXd mat_pow1(Eigen::MatrixXd base, unsigned int n){
        Eigen::MatrixXd id=Eigen::MatrixXd::Identity(base.rows(),base.cols());
        return n == 0 ? id : (base * mat_pow1 (base, n - 1));
    }

    inline void compare_score(search_tree* current){
        search_tree* root = current;
        for(; root->get_parent() != nullptr; root = root->get_parent());
        root->get_f_heap()->push(current);
    }

    inline vector<VectorXd> get_all_possibility(int n_input, double input[], int remain){
        vector<VectorXd> rtv;
        if(remain<2){
            for(int i=0; i<n_input; ++i){
                VectorXd temp=VectorXd::Zero(1);
                temp << input[i];
                rtv.push_back(temp);
            }
            return rtv;
        }
        vector<VectorXd> next_all_poss = get_all_possibility(n_input, input, remain-1);
        for(int i=0; i<n_input; ++i){
            for(int j=0; j < next_all_poss.size(); j++){
                VectorXd temp = VectorXd::Zero(remain);
                temp(0) = input[i];
                temp.block(1,0,remain-1,1) = next_all_poss[j];
                rtv.push_back(temp);
            }
        }
        return rtv;
    }

    inline void set_input_list_uav(vector<VectorXd> &input_list){
        double input[uav_ninput];
        for(int i=0; i<uav_ninput; i++){
            input[i]=(-(uav_ninput-1)/2+i)*uav_input_step;
        }
        input_list=get_all_possibility(uav_ninput, input, uav_input_length);
    }


    search_tree::search_tree(search_tree* parent, VectorXd input, VectorXd state, double time, double time_step,
                             vector<VectorXd> &input_list, unsigned int n_layers, double info_gain, double cost, double score){

        this->parent = parent;
        this->input  = input;
        this->state  = state;
        this->time   = time;
        this->time_step = time_step;
        this->info_gain = info_gain;
        this->cost   = cost;
        this->score  = score;
        this->max_node = 0;

        double body_vx=state(1);
        double body_vy=state(3);
        
        for(auto it = input_list.begin(); it != input_list.end(); ++it){
            
            if(n_layers > 1){

                VectorXd next_state=state;

                double next_score = 0.0;
                double next_info_gain=0.0;
                double next_cost=0.0;

                if(body_vx>uav_max_vx && (*it)(0)>0) continue;
                if(-body_vx>uav_max_vx && (*it)(0)<0) continue;
                
                if(body_vy>uav_max_vy && (*it)(1)>0) continue;
                if(-body_vy>uav_max_vy && (*it)(1)<0) continue;

                B.resize(state.size(), input.size());
                A.resize(state.size(), state.size());
                B<<set_UAV_B;
                A<<set_UAV_A;

                MatrixXd F=A*(time_step)+MatrixXd::Identity(state.size(),state.size());

                MatrixXd G=(A*(0.5*time_step)+MatrixXd::Identity(state.size(),state.size()))*(time_step)*B;
                next_state=F*state+G*(*it);

                body_vx = next_state(1);
                body_vy = next_state(3);

                if(body_vx>uav_max_vx && (*it)(0)>0) continue;
                if(-body_vx>uav_max_vx && (*it)(0)<0) continue;

                if(body_vy>uav_max_vy && (*it)(1)>0){
                    continue;
                }
                if(-body_vy>uav_max_vy && (*it)(1)<0) continue;
                double wx = next_state(x_in_state);
                double wy = next_state(y_in_state);

                if (explorer_->compAirGoalInfoGain(wx,wy,next_info_gain)){
                    next_cost = cost+(((*it).norm())*((*it).norm())+rho)*(time_step);
                    next_score = next_info_gain/next_cost;
                    search_tree* temp_tree= new search_tree(this, *it, next_state, time+time_step, time_step, input_list, n_layers-1, next_info_gain, next_cost,next_score);
                    this->children.push_back(temp_tree);
                }


            }

        }

        if(this->children.size()==0){
            compare_score(this);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>& path_search::viz_tree(visualization_msgs::Marker &tree_marker) {

        ROS_WARN("Time flag before tree viz");

        uav_tree_cloud_viz_.clear();
        viz_tree_recursive(uav_tree, tree_marker);

        tree_marker.header.frame_id = "/robot_1/map";
        tree_marker.header.stamp = ros::Time::now();
        explorer_->uav_tree_viz_pub_.publish(tree_marker);


        sensor_msgs::PointCloud2 tree_viz_output;
        pcl::toROSMsg(uav_tree_cloud_viz_, tree_viz_output);
        tree_viz_output.header.frame_id = "/robot_1/map";
        tree_viz_output.header.stamp = ros::Time::now();
        explorer_->uav_tree_point_viz_pub_.publish(tree_viz_output);




        ROS_WARN("Time flag after tree viz");
        return uav_tree_cloud_viz_;
    }

    void path_search::viz_tree_recursive(search_tree *tree, visualization_msgs::Marker &tree_marker){
        for (auto it = tree->get_children().rbegin(); it != tree->get_children().rend(); it++){
            geometry_msgs::Point parent, child;
            parent.z = child.z = 0;
            parent.x = (tree->get_state())(0);
            parent.y = (tree->get_state())(2);
            child.x  = ((*it)->get_state())(0);
            child.y  = ((*it)->get_state())(2);
            tree_marker.points.push_back(parent);
            tree_marker.points.push_back(child);

            this->viz_tree_recursive(*it, tree_marker);
        }


        pcl::PointXYZI tree_node_viz(100);
        tree_node_viz.x = (tree->get_state())(0);
        tree_node_viz.y = (tree->get_state())(2);
        uav_tree_cloud_viz_.push_back(tree_node_viz);
        

    }

    path_search::path_search(VectorXd uav_plan_pos, double plan_time_start){

        wx_min = explorer_->wx_min_;
        wx_max = explorer_->wx_max_;
        wy_min = explorer_->wy_min_;
        wy_max = explorer_->wy_max_;
      
        this->plan_time_start = plan_time_start;


        vector<VectorXd> input_list_uav;
        set_input_list_uav(input_list_uav);

        MatrixXd uav_A = MatrixXd::Zero(uav_plan_pos.size(), uav_plan_pos.size());
        MatrixXd uav_B = MatrixXd::Zero(uav_plan_pos.size(), input_list_uav[0].size());
        
        uav_A << set_UAV_A;
        uav_B << set_UAV_B;
        

        uav_tree = new root_node(uav_plan_pos, tree_time_step, input_list_uav, layers);

        visualization_msgs::Marker tree_marker;
        tree_marker.type=5;
        tree_marker.pose.orientation.w = 1.0;
        tree_marker.id = 0;
        tree_marker.type = visualization_msgs::Marker::LINE_LIST;
        tree_marker.scale.x = 0.005;
        tree_marker.color.r = 1.0;
        tree_marker.color.a = 0.3;

        viz_tree(tree_marker);
    }


    bool path_search::find_max_path(){

        search_tree* max_node=uav_tree->get_f_heap()->top();

        if(!max_node || max_node->get_score()==0.0) return false;
        
        //vector<geometry_msgs::Pose> way_pts;
        geometry_msgs::Pose single_way_pt;
        geometry_msgs::PoseArray way_pts_send;
        VectorXd way_state= VectorXd::Zero(4);

        single_way_pt.position.x = (max_node->get_state())(vx_in_state);
        single_way_pt.position.y = (max_node->get_state())(vy_in_state);
        single_way_pt.position.z = tree_time_step;
        way_pts_send.poses.push_back(single_way_pt);

        for(auto it=max_node; it; it=it->get_parent()){
            way_state=it->get_state();
            single_way_pt.position.x = way_state(x_in_state);
            single_way_pt.position.y = way_state(y_in_state);
            way_pts_send.poses.push_back(single_way_pt);
        }
        
        
        //way_pts_send.set_poses_vec(way_pts);
        explorer_->uav_waypts_pub_.publish(way_pts_send);
        return true;
    }
}

