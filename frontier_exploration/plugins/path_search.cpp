#include <frontier_exploration/path_search.h>
#include <boost/foreach.hpp>
#include <costmap_2d/cost_values.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>



//#define order 2
double tree_time_step;
#define layers 4
double collision_check_time_step;// tree_time_step / 5.0
#define ugv_max_vx 0.8
#define ugv_max_vy 0.5
#define ugv_max_omega 1

#define uav_max_vx 0.5
#define uav_max_vy 0.5

#define x_in_state 0
#define vx_in_state 1
#define y_in_state 2
#define vy_in_state 3
#define yaw_in_state 4
#define omega_in_state 5

double wx_min, wx_max, wy_min, wy_max;

double rho;

#define uav_input_length 2
#define uav_input_step 0.05
#define uav_ninput 5

#define ugv_input_length 3
#define ugv_input_step 0.1
#define ugv_ninput 3


#define set_UAV_A 0,1,0,0, 0,0,0,0, 0,0,0,1, 0,0,0,0
#define set_UAV_B 0,0, 1,0, 0,0, 0,1

#define set_UGV_A 0,1,0,0,0,0, 0,0,0,-omega,0,-vy, 0,0,0,1,0,0, 0,omega,0,0,0,vx, 0,0,0,0,0,1, 0,0,0,0,0,0
#define set_UGV_B 0,0,0, cos(yaw),-sin(yaw),0, 0,0,0, sin(yaw),cos(yaw),0, 0,0,0, 0,0,1

const double rho_array[3] = {1.0, 1.0, 1.0};

namespace frontier_exploration {
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
    inline void set_input_list_ugv(vector<VectorXd> &input_list){
        double input[ugv_ninput];
        for(int i=0; i<ugv_ninput; i++){
            input[i]=(-(ugv_ninput-1)/2+i)*ugv_input_step;
        }
        input_list=get_all_possibility(ugv_ninput, input, ugv_input_length);
    }

    inline void set_input_list_uav(vector<VectorXd> &input_list){
        double input[uav_ninput];
        for(int i=0; i<uav_ninput; i++){
            input[i]=(-(uav_ninput-1)/2+i)*uav_input_step;
        }
        input_list=get_all_possibility(uav_ninput, input, uav_input_length);
    }


    search_tree::search_tree(search_tree* parent, VectorXd input, VectorXd state, double time, double time_step,
                             vector<VectorXd> &input_list, unsigned int n_layers, double cost, double score, bool check_collision,
                             frontier_exploration::AirGroundSimExploreLayer* service, list<FrontierCell>* frontierA_cell_list){

        this->parent = parent;
        this->input  = input;
        this->state  = state;
        this->time   = time;
        this->time_step = time_step;
        this->cost   = cost;
        this->score  = score;
        this->max_node = 0;

        double omega=0;
        double yaw=0;
        double vx=state(1);
        double vy=state(3);
        if(state.size()>5){
            yaw=state(4);
            omega=state(5);
        }
        double body_vx = state(1)*cos(yaw)+state(3)*sin(yaw);
        double body_vy = -state(1)*sin(yaw)+state(3)*cos(yaw);

        //cout << "state" << state;
        // cout << "body_vy" << body_vy;

        //ROS_ERROR("st_check_1");
	/*
        if(check_collision){
            if(pow(state(vx_in_state),2) + pow(state(vy_in_state),2)>=pow(ugv_max_speed,2)){
                double input[(ugv_ninput+1)/2];
                for(int i=0; i<(ugv_ninput+1)/2; i++){
                    input[i]=(-(ugv_ninput-1)/2+i)*ugv_input_step;
                }
                input_list=get_all_possibility(ugv_ninput, input, ugv_input_length);

            }
            else set_input_list_ugv(input_list);
        }
        else{
            if(pow(state(vx_in_state),2)+pow(state(vy_in_state),2)>=pow(uav_max_speed,2)){
                double input[(uav_ninput+1)/2];
                for(int i=0; i<(uav_ninput+1)/2; i++){
                    input[i]=(-(uav_ninput-1)/2+i)*uav_input_step;
                }
                input_list=get_all_possibility(uav_ninput, input, uav_input_length);
            }
            else set_input_list_uav(input_list);

        }
        ROS_INFO("2");
	*/
        for(auto it = input_list.begin(); it != input_list.end(); ++it){

            FrontierCell temp_goal;
            temp_goal.score = 0.0;
            
            if(n_layers > 1){
                //check_collision/
                //
                bool collide = false;
                VectorXd next_state=state;
                
                MatrixXd G;
                MatrixXd F;

                double tmp_score = 0.0;

                if(check_collision){ // is ugv
                    VectorXd cur_input = *it;
                    cur_input(1)=cur_input(1)/2;

                    ("st_ugv_in");
                    if(body_vx>ugv_max_vx && (*it)(0)>0) continue;
                    if(-body_vx>ugv_max_vx && (*it)(0)<0) continue;

                    if(body_vy>ugv_max_vy && (*it)(1)>0){
                        continue;
                    }
                    if(-body_vy>ugv_max_vy && (*it)(1)<0) continue;

                    if(omega>ugv_max_omega && (*it)(2)>0) continue;
                    if(-omega>ugv_max_omega && (*it)(2)<0) continue;
                    
                    B.resize(state.size(), input.size());
                    A.resize(state.size(), state.size());
                    //for(double collision_check_time=collision_check_time_step; collision_check_time<=time+time_step;
                        //collision_check_time+=collision_check_time_step){
                    
                        B << set_UGV_B;
                        A << set_UGV_A;

                        if(!service->getExpA(vx, vy, omega, layers-n_layers+1, F)) {
                            //ROS_ERROR("V out of bound!");
                            F=A*time_step;
                            F=F.exp(); }//continue;
                        if(!service->get_integral_ExpA(vx, vy, omega, layers-n_layers+1, G)){
                            G=MatrixXd::Zero(state.size(),state.size());

                            for(int ord = 0; ord < 5; ++ord){
                                G = G+ std::pow(time_step,ord+1)/fac1(ord+1) * mat_pow1(A,ord);
                            }
                        } //continue;

                        //ROS_INFO_STREAM("For debug:" << "yaw: " << yaw << ", w:" << omega << ", vx:" << vx << ",vy " << vy);
                        //unsigned int i,j,k;
                        //service->getStateIndex(vx,vy,omega,i,j,k);
                        //ROS_INFO_STREAM("For debug:" << "i: " << i << ", j:" <<j << ", k:" << k);

                    //G=(F*B)*(std::exp(tree_time_step)-1);
                    //std::cout<<G<<std::endl;
                        //G = MatrixXd::Identity(next_state.size(),next_state.size())*tree_time_step+A*tree_time_step*tree_time_step/2;

                        G=G*B;
                    //std::cout<<F<<std::endl;
                    //std::cout<<"G:\n";
                        next_state=F  *next_state +G*cur_input;

                    double next_body_vx = next_state(1)*cos(next_state(4))+next_state(3)*sin(next_state(4));
                    double next_body_vy = -next_state(1)*sin(next_state(4))+next_state(3)*cos(next_state(4));
                    double next_omega = next_state(5);

                    if(next_body_vx>ugv_max_vx ) continue;
                    if(-next_body_vx>ugv_max_vx ) continue;

                    if(next_body_vy>ugv_max_vy ){
                        continue;
                    }
                    if(-next_body_vy>ugv_max_vy ) continue;

                    if(next_omega>ugv_max_omega ) continue;
                    if(-next_omega>ugv_max_omega) continue;
                    //std::cout<<"ugv_next_state\n";
                        double wx = next_state(x_in_state);
                        double wy = next_state(y_in_state);
                        unsigned int mx=0;
                        unsigned int my=0;
                        if (wx < wx_max && wx > wx_min && wy < wy_max && wy > wy_min){
                            service->getLayeredCostmapPtr()->getCostmap()->worldToMap(wx, wy, mx, my);
                            if (service->getLayeredCostmapPtr()->getCostmap()->getCost(mx,my) != costmap_2d::FREE_SPACE)
                                collide = true;
                            else{
                                temp_goal.frontier_cell_point.x = mx;
                                temp_goal.frontier_cell_point.y = my;
                                //double new_cost=cost+(((*it).norm())*((*it).norm())+rho)*(time_step);
				double new_cost = cost + (((*it).norm())*((*it).norm()) * (1.0 - rho_array[layers - n_layers]) / 0.15 + rho_array[layers - n_layers]) *(time_step);
                                //cout<<"new cost:"<<new_cost<<endl;
                                temp_goal.traverse_cost_ground = new_cost;//new_cost;
                                temp_goal.information_gain = service->getGroundInfoGain(wx, wy);
                                temp_goal.score = temp_goal.information_gain / temp_goal.traverse_cost_ground;
                            }
                        }
                        else{
                            collide=true;
                        }
                    //ROS_ERROR("st_ugv_prop");
                }

                else{ // is uav
                    if(body_vx>uav_max_vx && (*it)(0)>0) continue;
                    if(-body_vx>uav_max_vx && (*it)(0)<0) continue;
                    
                    if(body_vy>uav_max_vy && (*it)(1)>0) continue;
                    if(-body_vy>uav_max_vy && (*it)(1)<0) continue;
                    
                    //MatrixXd::Zero(state.size(),state.size());
                    B.resize(state.size(), input.size());
                    A.resize(state.size(), state.size());
                    B<<set_UAV_B;
                    A<<set_UAV_A;

                    MatrixXd F=A*(time_step)+MatrixXd::Identity(state.size(),state.size());
//                    for(int k=0; k<order; ++k) {
//                        MatrixXd temp=mat_pow(A*time_step,k)/fac(k);
//                        F=F+tem
//
// p;
//                        G=G+temp*tree_time_step/(k+1);
//                    }
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
                    unsigned int mx=0;
                    unsigned int my=0;
                    if (wx < wx_max-1 && wx > wx_min+1 && wy < wy_max-1 && wy > wy_min+1){
                        service->getLayeredCostmapPtr()->getCostmap()->worldToMap(wx, wy, mx, my);
                        temp_goal.frontier_cell_point.x = mx;
                        temp_goal.frontier_cell_point.y = my;
                        double new_cost=cost+(((*it).norm())*((*it).norm())+rho)*(time_step);
                            //cout<<"new cost:"<<new_cost<<endl;
                        temp_goal.traverse_cost_air = new_cost;


                            //cout << "wx:"<<next_state(x_in_state)<<"\nwy"<<next_state(y_in_state)<<"\n";
                    }
                    else{
                        collide=1;
                    }

                }
                if(!collide){

                    if(check_collision) {
                        //service->computeInfoGainAndScoreGround(temp_goal, *frontierA_cell_list);
                        //cout<<"new score UGV:"<<temp_goal.score<<endl;
                        //if(n_layers==2 && next_state(0)<-0.1 && next_state(2)<0.1 && temp_goal.score != 0)
                        //    std::cout<<"body_x2:\t"<<next_state(0)<<"\nbody_y2:\t"<<next_state(2)<<"\n"<<"score2:\n"<<temp_goal.score<<endl;
                        search_tree* temp_tree= new search_tree(this, *it, next_state, time+time_step, time_step+2 * tree_time_step, input_list,
                                                                n_layers-1, temp_goal.traverse_cost_ground,
                                                                temp_goal.score + score,check_collision, service, frontierA_cell_list);
                        this->children.push_back(temp_tree);

                    }
                    else{
                        service->computeInfoGainAndScoreAir(temp_goal);
                        //if(temp_goal.score!=0.0)cout<<"new score UAV:"<<temp_goal.score<<endl;
                        search_tree* temp_tree= new search_tree(this, *it, next_state, time+time_step, time_step+2 * tree_time_step, input_list,
                                                                n_layers-1, temp_goal.traverse_cost_air,
                                                                temp_goal.score + score,check_collision, service, frontierA_cell_list);
                        //if(temp_goal.score+ score!=0)cout<<"UAV_score:"<<temp_goal.score+ score<<endl;
                        this->children.push_back(temp_tree);
                    }


                }

            }
            //else if(check_collision) std::cout<<"body_x:\t"<<(this->get_state())(0)<<"\nbody_y:\t"<<(this->get_state())(2)<<"\n"<<"score:\n"<<this->get_score()<<endl;
        }

       if(this->children.size()==0){
            compare_score(this);
        }

        //cout << "cost:" << temp_goal.traverse_cost_ground << endl;
        //cout << "score:" << temp_goal.score + score << endl;
    }

    pcl::PointCloud<pcl::PointXYZI>& path_search::viz_tree(visualization_msgs::Marker &tree_marker) {

        ROS_WARN("Time flag before tree viz");

        uav_tree_cloud_viz_.clear();
        ugv_tree_cloud_viz_.clear();
        //viz_tree_recursive(uav_tree, false, tree_marker);
        viz_tree_recursive(ugv_tree, true,  tree_marker);

        sensor_msgs::PointCloud2 tree_viz_output;
        pcl::toROSMsg(ugv_tree_cloud_viz_, tree_viz_output);
        tree_viz_output.header.frame_id = "/robot_1/map";
        tree_viz_output.header.stamp = ros::Time::now();
        ugv_tree_viz_pub_.publish(tree_viz_output);


        ROS_WARN("Time flag after tree viz");
        return ugv_tree_cloud_viz_;
    }

    void path_search::viz_tree_recursive(search_tree *tree, bool is_ugv, visualization_msgs::Marker &tree_marker){
        for (auto it = tree->get_children().rbegin(); it != tree->get_children().rend(); it++){
            geometry_msgs::Point parent, child;
            parent.z = child.z = 0;
            parent.x = (tree->get_state())(0);
            parent.y = (tree->get_state())(2);
            child.x  = ((*it)->get_state())(0);
            child.y  = ((*it)->get_state())(2);
            tree_marker.points.push_back(parent);
            tree_marker.points.push_back(child);

            this->viz_tree_recursive(*it, is_ugv, tree_marker);
        }

        if (is_ugv){
            pcl::PointXYZI tree_node_viz(50);
            tree_node_viz.x = (tree->get_state())(0);
            tree_node_viz.y = (tree->get_state())(2);
            ugv_tree_cloud_viz_.push_back(tree_node_viz);
        }

        else{
            pcl::PointXYZI tree_node_viz(100);
            tree_node_viz.x = (tree->get_state())(0);
            tree_node_viz.y = (tree->get_state())(2);
            ugv_tree_cloud_viz_.push_back(tree_node_viz);
        }

    }

    path_search::path_search(VectorXd uav_plan_pos,VectorXd ugv_plan_pos, frontier_exploration::AirGroundSimExploreLayer* service,
                             std::list<FrontierCell>* frontierA_cell_list,  double wx_min_param, double wx_max_param, double wy_min_param,
                              double wy_max_param, double rho_param, double ground_range) : service_(service){
	
	ugv_input_pub_=nh_.advertise<geometry_msgs::Twist>("/ugv_acc", 10);

        wx_min = wx_min_param;
        wx_max = wx_max_param;
        wy_min = wy_min_param;
        wy_max = wy_max_param;

        rho = rho_param;

        tree_time_step = ground_range / 8.0;
        collision_check_time_step = 0.2;//tree_time_step / 5.0;

        //std::cout << "tree_time_step: " << tree_time_step << std::endl;
        
        plan_time_start = ros::Time::now().toSec();


        vector<VectorXd> input_list_ugv, input_list_uav;
        set_input_list_ugv(input_list_ugv);
        set_input_list_uav(input_list_uav);


        double omega=ugv_plan_pos(omega_in_state);
        double yaw=ugv_plan_pos(yaw_in_state);//same with current yaw.
        double vx=ugv_plan_pos(vx_in_state);
        double vy=ugv_plan_pos(vy_in_state);

        ROS_INFO_STREAM("For debug:" << "x:" << ugv_plan_pos(0) << ", y" << ugv_plan_pos(2)  << "\nyaw: " << yaw << ", w:" << omega << ", vx:" << vx << ",vy " << vy);

        MatrixXd uav_A = MatrixXd::Zero(uav_plan_pos.size(), uav_plan_pos.size());
        MatrixXd uav_B = MatrixXd::Zero(uav_plan_pos.size(), input_list_uav[0].size());
        MatrixXd ugv_A = MatrixXd::Zero(ugv_plan_pos.size(),ugv_plan_pos.size());
        MatrixXd ugv_B = MatrixXd::Zero(ugv_plan_pos.size(),input_list_ugv[0].size());

    	//ROS_INFO("check 3");
        uav_A << set_UAV_A;
        uav_B << set_UAV_B;
        ugv_A << set_UGV_A;
        ugv_B << set_UGV_B;

    	//ROS_INFO("check 4");

        uav_tree = new root_node(uav_plan_pos, 1, input_list_uav, layers,  0, service, frontierA_cell_list);
        ugv_tree = new root_node(ugv_plan_pos, 1, input_list_ugv, layers,  1, service, frontierA_cell_list);

	    //ROS_INFO("check 5");
        //cout << ugv_plan_pos << endl;

        ugv_tree_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tree",5);
	    //ROS_INFO("checkout");
    }


bool path_search::find_max_path(root_node* tree, MatrixXd &coef, std::list<FrontierCell> &node_list){

    TrajectoryGeneratorWaypoint P;
    
    search_tree* max_node=tree->get_f_heap()->top();
    //cout<<tree->get_f_heap()->size()<<endl;
    if(tree->get_state().size()>5) {
        for (; max_node && max_node->get_score() > 0.0; tree->get_f_heap()->pop()) {
            //cout << tree->get_f_heap()->size() << endl;
            if(tree->get_f_heap()->size()<1) return false;
            max_node = tree->get_f_heap()->top();

            //cout<<"max_score::"<<max_node->get_score()<<endl;
            if (max_node->get_state().size() < 5 || collision_free(max_node)) {
                break;
            }
        }
    }
    else{
        //cout<<"UAV_max_score:"<<max_node->get_score()<<endl;
    }

    if(!max_node || max_node->get_score()==0.0) return false;
    vector<VectorXd> way_state;
    vector<double> input_beta;

	geometry_msgs::Twist ugv_acc_;


    for(auto it=max_node; it; it=it->get_parent()){
        way_state.push_back(it->get_state());
        if((it->get_state().size())>5)
            input_beta.push_back((it->get_input())(2));
        // push node coords into list
        FrontierCell node_cell;
        unsigned int mx, my;
        service_->getLayeredCostmapPtr()->getCostmap()->worldToMap((it->get_state())(0), (it->get_state())(2), mx, my);
        node_cell.frontier_cell_point.x = mx;
        node_cell.frontier_cell_point.y = my;
        node_cell.score = it->get_score();
        //ROS_ERROR_STREAM("Max score:" << it->get_score());
        node_list.push_back(node_cell);

        /* pub acceleration
        if(it->get_parent()!=0 && it->get_parent()->get_parent()==0){
            ugv_acc_.linear.x=(it->get_input())(0);
            ugv_acc_.linear.y=(it->get_input())(1);
            ugv_acc_.linear.z=0;

            ugv_acc_.angular.x=0;
            ugv_acc_.angular.y=0;
            ugv_acc_.angular.z=(it->get_input())(2);
            ugv_acc = ugv_acc_;
        //ugv_input_pub_.publish(ugv_acc_);
        ROS_ERROR("pub once!!!");

        }*/
    }




    MatrixXd Path=MatrixXd::Zero(way_state.size(),3);
    MatrixXd Vel=MatrixXd::Zero(2,3);
    MatrixXd Acc=MatrixXd::Zero(2,3);
    VectorXd Time=VectorXd::Zero(way_state.size()-1);
    VectorXd beta=VectorXd::Zero(way_state.size()-1);
    VectorXd plan_time=Time;

    int i=0;
    for(auto rit = way_state.rbegin(); rit != way_state.crend(); ++rit){

        Path(i,0)=(*rit)(x_in_state);
        Path(i,1)=(*rit)(y_in_state);

        if (i == 0){

            Vel(0,0)=(*rit)(vx_in_state);
            Vel(0,1)=(*rit)(vy_in_state);
            Time(i)=(tree_time_step+i*2*tree_time_step);
		Time(i)*=1.5;
            plan_time(i)=Time(i)+this->get_plan_time_start();

            if((tree->get_state().size()) > 5)
                beta(i)=input_beta[i];
        }else if(i==way_state.size()-1){
            Vel(1,0)=(*rit)(vx_in_state);
            Vel(1,1)=(*rit)(vy_in_state);
        }else {
            Time(i)=(tree_time_step+i*2*tree_time_step);
		Time(i)*=1.5;
            plan_time(i)=plan_time(i-1)+Time(i);
            if((tree->get_state().size())>5)
            beta(i)=input_beta[i];
        }
        ++i;
    }
    coef=MatrixXd::Zero(way_state.size() - 1,20);
    coef.col(18) = beta;

    coef.col(19) = plan_time;
//    cout<<"Path0:\t"<<Path(0,0)<<"\t"<<Path(0,1)<<endl;
//    cout<<"Path1:\t"<<Path(1,0)<<"\t"<<Path(1,1)<<endl;
//    cout<<"Path2:\t"<<Path(2,0)<<"\t"<<Path(2,1)<<endl;
    //service_->traj_viz(Path,0,0,1);

    coef.block(0,0,way_state.size()-1,18) = P.PolyQPGeneration(Path,Vel,Acc,Time);

    return true;

}

bool path_search::collision_free(search_tree* max_node){

    // search_tree* root = max_node;
    // vector<search_tree*> tree_seq;
    FrontierCell temp_goal;
    // for(; root!= NULL; root = root->get_parent()){
    //     tree_seq.push_back(root);
    //     cout<<"time_step:"<<root->get_time_step()<<endl;
    // }
    // MatrixXd F,G;
    // VectorXd next_state;
    // VectorXd final_state;
    // double check_time=10;
    // //std::cout<<"tree_size:"<<tree_seq.size()<<"\n";
    // for(auto it=tree_seq.rbegin();it!=tree_seq.rend();++it){
    //     //int cnt=0;
    //     for(double collision_check_time=collision_check_time_step; collision_check_time<=check_time; collision_check_time+=collision_check_time_step){
    //         ++cnt;
    //         if(it==tree_seq.rbegin()) {
    //             //std::cout<<cnt<<"\n";
    //             break;
    //         }
    //         //std::cout<<cnt<<"\n";break;
    //         std::cout<<"chect_time:"<<check_time<<"\n";

    //         double omega=next_state(omega_in_state);
    //         double yaw=next_state(yaw_in_state);
    //         double vx=next_state(vx_in_state);
    //         double vy=next_state(vy_in_state);

    //         double body_vx=vx*cos(yaw)+vy*sin(yaw);
    //         double body_vy=-vx*sin(yaw)+vy*cos(yaw);

    //         //std::cout<<"body_vx:\t"<<body_vx<<"\nbody_vy:\t"<<body_vy<<"\n";
    //         unsigned int i,j,k;
    //         //service_->getExpA(vx, vy, omega, 0, F);

    //         //std::cout << "vx: " << vx << ", vy: " << vy << ", omega: " << omega << std::endl;

    //         //service_->get_integral_ExpA(vx, vy, omega, 0, G);
    //         //std::cout<<"F:\n"<<F<<"\n";
    //         MatrixXd A = MatrixXd::Zero(6,6);
    //         MatrixXd B = MatrixXd::Zero(6,3);
    //         A<<set_UGV_A;
    //         B<<set_UGV_B;

    //         if(!service_->getExpA(vx, vy, omega, 0, F)) {
    //             ROS_ERROR("check out of bound!");
    //             F=A*collision_check_time_step;
    //             F=F.exp(); }//continue;
    //         if(!service_->get_integral_ExpA(vx, vy, omega, 0, G)){
    //             G=MatrixXd::Zero(6,6);

    //             for(int ord = 0; ord < 5; ++ord){
    //                 G = G+ std::pow(collision_check_time_step,ord+1)/fac1(ord+1) * mat_pow1(A,ord);
    //             }
    //         } //continue;

    //         //F = *F_ptr;
    //         //G = MatrixXd::Identity(next_state.size(),next_state.size())*collision_check_time_step+collision_check_time_step*collision_check_time_step/2*A;
    //         G=G*B;
    //         VectorXd cur_input = (*it)->get_input();
    //         cur_input(1)=cur_input(1)/2;
            
    //         next_state=F*next_state+G*cur_input;

    //         double wx= next_state(x_in_state);
    //         double wy= next_state(y_in_state);
    //         unsigned int mx = 0;
    //         unsigned int my = 0;
    //         if (wx < wx_max && wx > wx_min && wy < wy_max && wy > wy_min){
    //             service_->getLayeredCostmapPtr()->getCostmap()->worldToMap(wx, wy, mx, my);
    //             if (service_->getLayeredCostmapPtr()->getCostmap()->getCost(mx,my) != costmap_2d::FREE_SPACE){
    //                 return false;
    //             }
    //             else{
    //                 temp_goal.frontier_cell_point.x = mx;
    //                 temp_goal.frontier_cell_point.y = my;
    //                 temp_goal.traverse_cost_ground = (*it)->get_cost()+((((*it)->get_input()).norm())*(((*it)->get_input()).norm())+rho)*(*it)->get_time_step();
    //             }
    //         }
    //         else{
    //             return false;
    //         }

    //     }
    //     final_state=next_state;
    //     next_state=(*it)->get_state();
    //     check_time=(*it)->get_time_step();
    //     //if(check_time==1) cout<<next_state<<endl;
    //     //cout<<"set check time:"<<check_time<<endl;
    // }



	vector<VectorXd> way_state;
    vector<double> input_beta;
    MatrixXd coef;
    TrajectoryGeneratorWaypoint P;


    for(auto it=max_node; it; it=it->get_parent()){
        way_state.push_back(it->get_state());
        if((it->get_state().size())>5)
            input_beta.push_back((it->get_input())(2));
    }
    MatrixXd Path=MatrixXd::Zero(way_state.size(),3);
    MatrixXd Vel=MatrixXd::Zero(2,3);
    MatrixXd Acc=MatrixXd::Zero(2,3);
    VectorXd Time=VectorXd::Zero(way_state.size()-1);
    VectorXd beta=VectorXd::Zero(way_state.size()-1);
    VectorXd plan_time=Time;

    int i=0;
    for(auto rit = way_state.rbegin(); rit != way_state.crend(); ++rit){

        Path(i,0)=(*rit)(x_in_state);
        Path(i,1)=(*rit)(y_in_state);

        if (i == 0){

            Vel(0,0)=(*rit)(vx_in_state);
            Vel(0,1)=(*rit)(vy_in_state);
            Time(i)=(tree_time_step+i*2*tree_time_step);
		Time(i)*=1.5;
            plan_time(i)=Time(i)+this->get_plan_time_start();

        }else if(i==way_state.size()-1){
            Vel(1,0)=(*rit)(vx_in_state);
            Vel(1,1)=(*rit)(vy_in_state);
        }else {
            Time(i)=(tree_time_step+i*2*tree_time_step);
            	Time(i)*=1.5;
	    plan_time(i)=plan_time(i-1)+Time(i);
        }
        ++i;
    }
    coef=P.PolyQPGeneration(Path,Vel,Acc,Time);

    double wx=0,wy=0;
    unsigned int mx = 0;
    unsigned int my = 0;
	for(double dT = this->get_plan_time_start();dT< plan_time(plan_time.rows()-1);dT+=0.2) {
        for (int i = 0; i < plan_time.size(); i++) {
            if (dT < plan_time(i)) {
                double tt = i > 0 ? dT - plan_time(i - 1) : dT - this->get_plan_time_start();
                        //std::cout<<tt<<'\n';
                        //std::cout<<"T(i-1): "<<T(i-1)<<'\n';

                Eigen::Matrix<double, 1, 6> t_p;
                t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);

                Eigen::VectorXd coef_x;
                Eigen::VectorXd coef_y;
                coef_x = (coef.block(i, 0, 1, 6)).transpose();
                coef_y = (coef.block(i, 6, 1, 6)).transpose();

                wx = t_p * coef_x;
                wy = t_p * coef_y;

                if (wx < wx_max && wx > wx_min && wy < wy_max && wy > wy_min){
                	service_->getLayeredCostmapPtr()->getCostmap()->worldToMap(wx, wy, mx, my);
                	if (service_->getLayeredCostmapPtr()->getCostmap()->getCost(mx,my) != costmap_2d::FREE_SPACE){
                    	return false;
                	}
            	}
            	else{
                	return false;
            	}



                break;

            }
        }
    }


    //coef.col(18) = beta;

    //coef.col(19) = plan_time;
//    cout<<"Path0:\t"<<Path(0,0)<<"\t"<<Path(0,1)<<endl;
//    cout<<"Path1:\t"<<Path(1,0)<<"\t"<<Path(1,1)<<endl;
//    cout<<"Path2:\t"<<Path(2,0)<<"\t"<<Path(2,1)<<endl;


    // std::cout << "vx: " << final_state(1) << ", vy: " << final_state(3) << std::endl;
    // cout<<"max_node_v:\n"<<(max_node->get_state())(1)<<endl<<(max_node->get_state())(3)<<endl;
    ROS_WARN("Finished checking collision free!");
    return true;
}

}

