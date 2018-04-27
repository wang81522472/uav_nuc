#include <duet_exploration/harmonic_duet.h>
#include <duet_exploration/frontier_search.h>
#include "duet_exploration/trajectory_generator_lite.h"
#include "duet_exploration/bezier_base.h"
#include "duet_exploration/dataType.h"
#include <queue>

#define FRONTIER_B_VAL -1
#define OBSTACLE_VAL -2
#define UAV_VAL -3
#define UGV_VAL -5
#define UNKNOWN_VAL -4
#define OUT_OF_BOUND_VAL -6
#define FREE_VAL 0
#define TYPE_UGV 0
#define TYPE_UAV_FRONA 1
#define TYPE_UAV_FRONB 2
#define TYPE_UAV_COMB 3 // combined field
#define UGV_MAX_SPEED 0.5
static const double PI = 3.1415927;
static const int JACOBI_METHOD = 0, GS_METHOD = 1, SOR_METHOD = 2;
static const double INFO_UNKNWON = 0.01, INFO_FRONA = 0.1, INFO_FRONB = 0.89;

MatrixXd _PolyCoeff;
MatrixXd _MQM;
VectorXd _C, _Cv, _Ca, _Cj;
int _segment_num;
int _minimize_order = 3;
int _traj_order = 8;
int _traj_id    = 1;
double _vis_traj_width = 0.1;
quadrotor_msgs::PolynomialTrajectory _traj;
TrajectoryGenerator _trajectoryGenerator;
ros::Time _start_time;
VectorXd _Time;
bool traj_finish=0;
double _check_horizon=10.0;

ofstream mp;
ofstream bts;
ofstream btf;

ros::Time explore_start_time;


duet_exploration::DuetExploration * explorer_;//("harmonic_duet_node", "/robot_1/odom");
namespace duet_exploration
{   
    bool isContains(Cube2D cube1, Cube2D cube2)
    {
        if( cube1.vertex(0, 0) >= cube2.vertex(0, 0) && 
            cube1.vertex(0, 1) <= cube2.vertex(0, 1) &&
            cube1.vertex(2, 0) <= cube2.vertex(2, 0) && 
            cube1.vertex(2, 1) >= cube2.vertex(2, 1) )
            return true;
        else
            return false; 
    }

    Cube2D generateCube( Vector2d pt) 
    {   
    /*
            P4------------P3 
             |             |              
             |             |             
            P1------------P2              
                
             |--------> y
             | 
             |
             x
    */    
        Cube2D cube;
        double x_u = pt(0);
        double x_l = pt(0);
        
        double y_u = pt(1);
        double y_l = pt(1);

        cube.vertex.row(0) = Vector2d(x_u, y_l);  
        cube.vertex.row(1) = Vector2d(x_u, y_u);  
        cube.vertex.row(2) = Vector2d(x_l, y_u);  
        cube.vertex.row(3) = Vector2d(x_l, y_l);  
        return cube;
    }

    void corridorSimplify(vector<Cube2D> & cubicList)
    {
        vector<Cube2D> cubicSimplifyList;
        for(int j = 0; j < (int)cubicList.size(); j++)
        {   
            for(int k = j+1; k < (int)cubicList.size(); k++)
            {   
                if(cubicList[k].valid == false)
                    continue;
                else if(isContains(cubicList[j], cubicList[k]))
                    cubicList[k].valid = false;   
            }
        }

        for(auto cube:cubicList)
            if(cube.valid == true)
                cubicSimplifyList.push_back(cube);

        cubicList = cubicSimplifyList;
    }

    Cube2D inflate(Cube2D cube, Cube2D lstcube)
    {   
        Cube2D cubeMax = cube;

        //cube.printBox();

        double pt_max_x, pt_min_x, pt_max_y, pt_min_y;
        pt_max_x = explorer_->wx_max_;
        pt_min_x = explorer_->wx_min_;
        pt_max_y = explorer_->wy_max_;
        pt_min_y = explorer_->wy_min_;

        int id_x_max = explorer_->map_inflation_->map_ptr_->size_x_;
        int id_y_max = explorer_->map_inflation_->map_ptr_->size_y_;

        unsigned int id_x = 0, id_y = 0;
        // Inflate sequence: left, right, front, back
        MatrixXi vertex_idx(4, 2);
        for (int i = 0; i < 4; i++)
        {   
            double coord_x = max(min(cube.vertex(i, 0), pt_max_x), pt_min_x);
            double coord_y = max(min(cube.vertex(i, 1), pt_max_y), pt_min_y);

            Vector2i pt_idx;
            if(explorer_->map_inflation_->map_ptr_->worldToMap(coord_x, coord_y, id_x, id_y))
            {   
                pt_idx << id_x, id_y;
                if(explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::LETHAL_OBSTACLE ||
                   explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::INSCRIBED_INFLATED_OBSTACLE )
                    return cubeMax;
            }
            else
                return cubeMax;

            vertex_idx.row(i) = pt_idx;
        }

    /*
            P4------------P3 
             |             |              
             |             |             
            P1------------P2              
                
             |--------> y
             | 
             |
             x
    */              
    // now is the left side : (p1 -- p4) line sweep
    // ############################################################################################################

        //cout<<"fuck "<<endl;
        bool loop  = true;    
        int  step_length = 4;

        MatrixXi vertex_idx_lst = vertex_idx;

        int max_iter = 100;
        int iter = 0;
        while(iter < max_iter)
        {   
            int y_lo = max(0, vertex_idx(0, 1) - step_length);
            int y_up = min(id_y_max, vertex_idx(1, 1) + step_length);

            for(id_y = vertex_idx(0, 1); id_y >= y_lo; id_y-- )
            {   
                if( loop == false) 
                    break;
                
                for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
                {    
                    if(explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::LETHAL_OBSTACLE ||
                    explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::INSCRIBED_INFLATED_OBSTACLE )
                    {
                        loop = false;
                        break;
                    }
                }
            }

            vertex_idx(0, 1) = min(id_y+2, (unsigned int)vertex_idx(0, 1));
            vertex_idx(3, 1) = min(id_y+2, (unsigned int)vertex_idx(3, 1));

            //ROS_WARN("finished the 1st face");
        // now is the right side : (p2 -- p3) line
        // ############################################################################################################
            loop = true;
            for(id_y = vertex_idx(1, 1); id_y < y_up; id_y++ )
            {   
                if( loop == false) 
                    break;
                
                for(id_x = vertex_idx(1, 0); id_x >= vertex_idx(2, 0); id_x-- )
                {
                    if(explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::LETHAL_OBSTACLE ||
                   explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::INSCRIBED_INFLATED_OBSTACLE )
                    {
                        loop = false;
                        break;
                    }
                }
            }

            //ROS_WARN("finish the loop of the 2nd face");
            vertex_idx(1, 1) = max(id_y-2, (unsigned int)vertex_idx(1, 1));
            vertex_idx(2, 1) = max(id_y-2, (unsigned int)vertex_idx(2, 1));

        // now is the front side : (p1 -- p2) line
        // ############################################################################################################
            int x_lo = max(0, vertex_idx(3, 0) - step_length);
            int x_up = min(id_x_max, vertex_idx(0, 0) + step_length);
            loop = true;
            for(id_x = vertex_idx(0, 0); id_x < x_up; id_x++ )
            {   
                if( loop == false) 
                    break;
                
                for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
                {
                    if(explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::LETHAL_OBSTACLE ||
                   explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::INSCRIBED_INFLATED_OBSTACLE )
                    {
                        loop = false;
                        break;
                    }
                }
            }

            //ROS_WARN("finish the loop of the 3rd face");
            vertex_idx(0, 0) = id_x-2;
            vertex_idx(1, 0) = id_x-2;

        // now is the back side : (p4 -- p3) line
        // ############################################################################################################
            loop = true;
            for(id_x = vertex_idx(3, 0); id_x >= x_lo; id_x-- )
            {   
                if( loop == false) 
                    break;
                
                for(id_y = vertex_idx(3, 1); id_y <= vertex_idx(2, 1); id_y++ )
                {
                    if(explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::LETHAL_OBSTACLE ||
                   explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::INSCRIBED_INFLATED_OBSTACLE )
                    {
                        loop = false;
                        break;
                    }
                }
            }

            //ROS_WARN("finish the loop of the 4th face");
            vertex_idx(3, 0) = id_x+2;
            vertex_idx(2, 0) = id_x+2;

            if(vertex_idx_lst == vertex_idx)
            {   
                //cout<<"finish"<<endl;
                break;
            }

            vertex_idx_lst = vertex_idx;

            //cout<<"idx: \n"<<vertex_idx<<endl;
            MatrixXd vertex_coord(4, 2);
            for(int i = 0; i < 4; i++)
            {   
                double coord_x, coord_y;
                int idx_x = max(min(vertex_idx(i, 0), id_x_max-1), 0);
                int idx_y = max(min(vertex_idx(i, 1), id_y_max-1), 0);

                explorer_->map_inflation_->map_ptr_->mapToWorld(idx_x, idx_y, coord_x, coord_y);
                Vector2d pos(coord_x, coord_y); 
                
                vertex_coord.row(i) = pos;
            }
            //cout<<"coord: \n"<<vertex_coord<<endl;

            cubeMax.setVertex(vertex_coord);
            if(isContains(lstcube, cubeMax))
                return lstcube;

            iter ++;
        }

        return cubeMax;
    }
    void time_alloc(vector<Cube2D>& cube_list, Path2D& path){
        if(cube_list.size()==1){
            cube_list[0].t= (Vector2d(path.back()[0] - path.front()[0], path.back()[1]-path.front()[1]).norm())/UGV_MAX_SPEED/0.7;
            return;
        }
        for(int i=0; i<cube_list.size(); i++){
            cube_list[i].t= i==cube_list.size()-1 ? ((cube_list[i].center-Vector2d(path.back()[0],path.back()[1])).norm())/UGV_MAX_SPEED/0.7 : ((cube_list[i+1].center-cube_list[i].center).norm())/UGV_MAX_SPEED/0.7;
            if (cube_list[i].t==0) cube_list[i].t=0.1;
        }

    }

    vector<Cube2D> corridorGeneration(Path2D& path)
    {
        vector<Cube2D> cubeList;
        array<double, 2> state;
        Vector2d pt;

        Cube2D lstcube;
        int path_sz=(int)path.size();

        for (int i = 0; i < (int)path.size(); i += 1)
        {
            state = path[i];
            pt(0) = state[0];
            pt(1) = state[1];

            Cube2D cube = generateCube(pt);
            cube.center = pt;
            cube = inflate(cube, lstcube);

            //cube.printBox();
            //ROS_BREAK();

            bool is_cut = false;

            for(int i = 0; i < 2; i ++)
                if( cube.box[i].second - cube.box[i].first < 4 * explorer_->resolution_ && (pt-Vector2d(path.front()[0],path.front()[1])).norm()>2.0)
                    is_cut = true;

            if( is_cut){
                for(int cut_idx=0; cut_idx<path_sz-i+2; cut_idx++) path.pop_back();
                break;
            }
            
            lstcube = cube;

            //cube.t = time[i];
            //cube.t = 2*(cube.center.transpose()-cube.vertex.row(0)).norm()/UGV_MAX_SPEED/0.7;//2.0; // allocate a time duration 
            cubeList.push_back(cube);
        }

        //ROS_WARN("Corridor generated, size is %d", (int)cubeList.size() );
        corridorSimplify(cubeList);
        time_alloc(cubeList, path);
        for(auto ptr:cubeList)
        {
            //ptr.printBox();
            cout<<ptr.t<<endl;
            cout<<"path size: "<<path.size()<<endl;
        }

        //ROS_WARN("Corridor simplified, size is %d", (int)cubeList.size());

        return cubeList;
    }

    quadrotor_msgs::PolynomialTrajectory getBezierTraj()
    {
        quadrotor_msgs::PolynomialTrajectory traj;
          traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
          //int _segment_num = _PolyCoeff.rows();
          traj.num_segment = _segment_num;

          int order = _traj_order;
          int poly_num1d = order + 1;
          int polyTotalNum = _segment_num * (order + 1);

          traj.coef_x.resize(polyTotalNum);
          traj.coef_y.resize(polyTotalNum);
          traj.coef_z.resize(polyTotalNum);

          //cout<<"row:"<<_PolyCoeff.rows()<<"\ncol:"<<_PolyCoeff.cols()<<"\npolyTotalNum:"<<polyTotalNum<<endl;
          int idx = 0;
          for(int i = 0; i < _segment_num; i++ )
          {    
              for(int j =0; j < poly_num1d; j++)
              { 
                  traj.coef_x[idx] = _PolyCoeff(i,                  j);
                  traj.coef_y[idx] = _PolyCoeff(i,     poly_num1d + j);
                  //traj.coef_z[idx] = _PolyCoeff(i, 2 * poly_num1d + j);
                  idx++;
              }
          }


          traj.header.frame_id = "/bernstein";
          traj.header.stamp = explorer_->ugv_odom_.header.stamp; //ros::Time(_odom.header.stamp.toSec()); 
          _start_time = traj.header.stamp;

          traj.time.resize(_segment_num);
          traj.order.resize(_segment_num);
          traj.mag_coeff = 1.0;
          for (int idx = 0; idx < _segment_num; ++idx){
              traj.time[idx] = _Time(idx);
              traj.order[idx] = _traj_order;
          }

          traj.start_yaw = 0.0;
          traj.final_yaw = 0.0;

          traj.trajectory_id = _traj_id;
          traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
          return traj;
    }

    vector<double> getPosFromBezier(const MatrixXd & polyCoeff, double t_now, int seg_now )
    {
        vector<double > ret(2, 0);
        VectorXd ctrl_now = polyCoeff.row(seg_now);
        int ctrl_num1D = polyCoeff.cols() / 2;

        for(int i = 0; i < 2; i++)
            for(int j = 0; j < ctrl_num1D; j++)
                ret[i] += _C(j) * ctrl_now(i * ctrl_num1D + j) * pow(t_now, j) * pow((1 - t_now), (_traj_order - j) ); 

        return ret;  
    }

    void visBezierTrajectory(MatrixXd polyCoeff, VectorXd time)
    {   
        visualization_msgs::Marker traj_vis;

        traj_vis.header.stamp       = ros::Time::now();
        traj_vis.header.frame_id    = "/robot_1/odom";

        traj_vis.ns = "trajectory/trajectory";
        traj_vis.id = 0;
        traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
        
        traj_vis.action = visualization_msgs::Marker::DELETE;
        explorer_->ugv_checkTraj_vis_pub_.publish(traj_vis);

        traj_vis.action = visualization_msgs::Marker::ADD;
        traj_vis.scale.x = _vis_traj_width;
        traj_vis.scale.y = _vis_traj_width;
        traj_vis.scale.z = _vis_traj_width;
        traj_vis.pose.orientation.x = 0.0;
        traj_vis.pose.orientation.y = 0.0;
        traj_vis.pose.orientation.z = 0.0;
        traj_vis.pose.orientation.w = 1.0;
        traj_vis.color.r = 1.0;
        traj_vis.color.g = 0.0;
        traj_vis.color.b = 0.0;
        traj_vis.color.a = 0.6;

        double traj_len = 0.0;
        int count = 0;
        Vector3d cur, pre;
        cur.setZero();
        pre.setZero();
        
        traj_vis.points.clear();

        vector<double> state;
        geometry_msgs::Point pt;

        int segment_num  = polyCoeff.rows();
        for(int i = 0; i < segment_num; i++ ){
            for (double t = 0.0; t < 1.0; t += 0.05 / time(i), count += 1){
                state = getPosFromBezier( polyCoeff, t, i );
                cur(0) = pt.x = time(i) * state[0];
                cur(1) = pt.y = time(i) * state[1];
                //cur(2) = pt.z = time(i) * state[2];
                traj_vis.points.push_back(pt);

                if (count) traj_len += (pre - cur).norm();
                pre = cur;
            }
        }

        ROS_INFO("[GENERATOR] The length of the trajectory; %.3lfm.", traj_len);
        explorer_->ugv_traj_viz_pub_.publish(traj_vis);
    }

    bool checkPointOccupied(vector<double> check_pt)
    {
        unsigned int id_x=0,id_y=0;

        if( !explorer_->map_inflation_->map_ptr_->worldToMap(check_pt[0], check_pt[1], id_x, id_y) ||explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::LETHAL_OBSTACLE ||
                   explorer_->map_inflation_->map_ptr_->getCostInflated(id_x, id_y)  == simple_map_2d::INSCRIBED_INFLATED_OBSTACLE )
            return true;
        else
            return false;
    }

    bool checkHalfWay()
    {   
        if(!traj_finish) return false;

        ros::Time time_1 = ros::Time::now();
        vector<double> check_traj_pt;
        vector<double> state;

        geometry_msgs::Point pt;

        double t_s = max(0.0, (explorer_->ugv_odom_.header.stamp - _start_time).toSec());      
        int idx;
        for (idx = 0; idx < _segment_num; ++idx){
          if (t_s > _Time(idx) && idx + 1 < _segment_num)
              t_s -= _Time(idx);
          else break;
        }

        double duration = 0.0;
        double t_ss;
        check_traj_pt.resize(2);
        for(int i = idx; i < _segment_num; i++ )
        {
            t_ss = (i == idx) ? t_s : 0.0;
            for(double t = t_ss; t < _Time(i); t += 0.01){
                double t_d = duration + t - t_ss;
                if( t_d > _check_horizon ) break;
                state = getPosFromBezier( _PolyCoeff, t/_Time(i), i );
                pt.x = check_traj_pt[0] = _Time(i) * state[0]; 
                pt.y = check_traj_pt[1] = _Time(i) * state[1];
                
                if( checkPointOccupied(check_traj_pt))
                {   
                    ROS_ERROR("predicted collision time is %f ahead", t_d);
                    return true;
                }
          }

          duration += _Time(i) - t_ss;
        }

        ros::Time time_2 = ros::Time::now();
        ROS_WARN("Time in collision checking is %f", (time_2 - time_1).toSec());

        return false;
    }

    using simple_map_2d::LETHAL_OBSTACLE;
    using simple_map_2d::NO_INFORMATION;
    using simple_map_2d::FREE_SPACE;
    using simple_map_2d::INSCRIBED_INFLATED_OBSTACLE;

    using std::ofstream;

    bool DuetExploration::explore_main_loop(){

        ofstream time_file, progress_file;
        time_file.open("/home/gf/Documents/explore_progress_log/time_log.txt");
        progress_file.open("/home/gf/Documents/explore_progress_log/prog_log.txt");

        // for test bfs performance
        ofstream iter_cnt_file1, iter_cnt_file2;
        iter_cnt_file1.open("/home/gf/Documents/explore_progress_log/cnt_log1.txt");
        iter_cnt_file2.open("/home/gf/Documents/explore_progress_log/cnt_log2.txt");

        while (ros::ok())
        {
            double iter_start_time = ros::Time::now().toSec();
            ROS_WARN("*** Flag start loop");

            copyMapsData();

            double explore_progress = countProgress(time_file, progress_file);
            if (!initCharmapInfo() && (explore_progress > 0.8)){ // generate charmap_info_
                break; // exploration complete
            }
            ROS_INFO("Flag init charmap_info");

            // ******* UGV Harmonic Exploration *******

            if (is_has_ugv_)
            {
                double update_change, iter_update_precision = update_precision_ / sqrt(explore_progress) * 2.0;

                initHarmonicFieldUGV();
                ROS_INFO("Flag init Harmonic Fields");
                if (is_iter_bfs_)
                    getBfsSeq();
                unsigned int iter_cnt = 0;

                // for test bfs performance
                std::vector<double> harmonic_field_test = harmonic_field_ugv_;
                do {
                    iterUpdate(harmonic_field_test, update_change, TYPE_UGV);
                    iter_cnt ++;
                }while(update_change > iter_update_precision); //|| iter_cnt < (ground_range_ + 1.0) / resolution_)
                ROS_INFO_STREAM("### Harmonic converge (" << update_change << ") after " << iter_cnt << " iterations");
                
                if (!new_ugv_boundary_idx_vec_.empty())
                    iter_cnt_file1 << iter_cnt << std::endl;
                //TODO: delete these code!!!
                iter_cnt = 0;

                do {
                    if (is_iter_bfs_ && !new_ugv_boundary_idx_vec_.empty())// compute field using bfs_method
                        iterUpdateBFS(harmonic_field_ugv_, update_change);
                    else{
                        //ROS_WARN("No new boundary! Using normal update-sequence!");
                        iterUpdate(harmonic_field_ugv_, update_change, TYPE_UGV);
                    }
                    iter_cnt ++;
                }while(update_change > iter_update_precision);// || iter_cnt < (ground_range_ + 1.0) / resolution_);
                ROS_INFO_STREAM("### Harmonic converge (" << update_change << ") after " << iter_cnt << " iterations");
                
                if (!new_ugv_boundary_idx_vec_.empty())
                    iter_cnt_file2 << iter_cnt << std::endl;

                compFieldGradient(harmonic_field_ugv_, harmonic_grad_ugv_, TYPE_UGV);

                for( int lp_cnt=0; !gradDescend(harmonic_grad_ugv_, 300, TYPE_UGV) && lp_cnt<5; lp_cnt++) {
                    if(lp_cnt==4){
                        ROS_ERROR("UGV trajectory cannot solve!");
                        _traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
                        explorer_->ugv_traj_pub_.publish(_traj);
                    }
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                };

                pubGrad(harmonic_grad_ugv_, ugv_grad_pub_);
                visualize_field(harmonic_field_ugv_, harmonic_field_ugv_viz_, harmonic_field_ugv_viz_pub_);

                utilDiscount();
            }
            // ******* UAV Fast-Frontier Exploration *******
            if (is_has_uav_)
            {
                procPickPubUAVGoal();
                uav_goal_.point.z=2;
                for(int lp_cnt=0; !motion_primitive() && lp_cnt<3; lp_cnt++){
                    if(lp_cnt==2) {
                        uav_goal_.point.z=-1;
                        ROS_ERROR("UAV motion primitive failed!");
                        break;
                    }
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                    ros::spinOnce();
                }
            }
            visualize_frontier();

            while(ros::ok && (ros::Time::now().toSec() - iter_start_time < update_duration_)){
                ros::spinOnce();
                if (checkHalfWay() || ros::Time::now().toSec() - iter_start_time >= update_duration_)
                    break;
                copyMapsData();
                ros::Duration(0.1).sleep();
            }
        }
    }

    double angle_diff(double a, double b){ //compute angle (in rads) subtraction
        double diff = fabs(a - b);
        if (diff > PI)
            diff -= PI;
        return diff;
    }

    std::vector<unsigned int> harmonic_nhood4(unsigned int idx, unsigned int size_x_, unsigned int size_y_){
        std::vector<unsigned int> out;
        if (idx > size_x_ * size_y_ -1){
            ROS_WARN("Evaluating nhood for offmap point");
            return out;
        }

        if(idx % size_x_ > 0){
            out.push_back(idx - 1);
        }
        if(idx % size_x_ < size_x_ - 1){
            out.push_back(idx + 1);
        }
        if(idx >= size_x_){
            out.push_back(idx - size_x_);
        }
        if(idx < size_x_*(size_y_-1)){
            out.push_back(idx + size_x_);
        }
        return out;
    }

    DuetExploration::DuetExploration(std::string node_name, std::string global_frame_name)
            : nh_("~")
            , node_name_(node_name)
            , global_frame_name_(global_frame_name)
    {
        simple_map_ = boost::shared_ptr<simple_map_2d::SimpleMap2DROS>(
                new simple_map_2d::SimpleMap2DROS("simple_map", global_frame_name_));
        map_inflation_ = boost::shared_ptr<simple_map_2d::MapInflation>(
                new simple_map_2d::MapInflation(simple_map_->map_));

        // ROS Subscribers init
        uav_odom_sub_ = nh_.subscribe("simple_map/uav_odom",4,&DuetExploration::uav_odom_call_back,this);
        ugv_odom_sub_ = nh_.subscribe("simple_map/ugv_odom",4,&DuetExploration::ugv_odom_call_back,this);

        // ROS Publishers init
        frontier_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
        corridor_vis_pub_   = nh_.advertise<visualization_msgs::MarkerArray>("corridor_vis", 1);

        uav_tree_point_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("uav_tree_point",5);
        uav_tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("uav_tree",5);
        uav_waypts_pub_ = nh_.advertise<geometry_msgs::PoseArray>("uav_waypoints",5);

        gridmap_no_inflation_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gridmap_no_inflation", 1);
        harmonic_field_ugv_viz_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("harmonic_field_ugv_viz", 1);
        harmonic_field_uav_fronA_viz_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("harmonic_field_uav_viz", 1);
        ugv_path_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("ugv_path_viz", 1);
        uav_path_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("uav_path_viz", 1);
        ugv_grad_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("ugv_gradient_viz", 1);
        uav_grad_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("uav_gradient_viz", 1);
        ugv_grad_pub_ = nh_.advertise<duet_exploration::MultiArrayWithHeader>("ugv_gradient", 3);
        uav_grad_pub_ = nh_.advertise<duet_exploration::MultiArrayWithHeader>("uav_gradient", 3);
        uav_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("uav_goal_point", 3);
        ugv_traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("ugv_poly_traj",3);
        ugv_traj_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("ugv_poly_traj_viz",3);
        ugv_checkTraj_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("ugv_check_traj_viz",3);

        // System Params
        nh_.param<bool>("/is_sim", is_sim_, true);
        nh_.param<bool>("/is_has_uav", is_has_uav_, true);
        nh_.param<bool>("/is_has_ugv", is_has_ugv_, true);

        nh_.param<int>("min_frontier_size", min_frontier_size_, 5);

        nh_.param<double>("/ground_range", ground_range_, 8.0);
        nh_.param<double>("UAV_fov_x", UAV_fov_x_, 0.6);
        nh_.param<double>("UAV_fov_y", UAV_fov_y_, 0.6);
        nh_.param<double>("obstacle_height", obstacle_height_, 0.6);
        nh_.param<double>("uav_height", uav_height_, 2.0);
        nh_.param<double>("/wx_min", wx_min_, -1.0);
        nh_.param<double>("/wx_max", wx_max_, 19.0);
        nh_.param<double>("/wy_min", wy_min_, -1.0);
        nh_.param<double>("/wy_max", wy_max_, 19.0);
        nh_.param<double>("update_duration", update_duration_, 2.0);

        nh_.param<double>("/ugv_tf_offset_x", ugv_tf_offset_x_, 0.0);
        nh_.param<double>("/ugv_tf_offset_y", ugv_tf_offset_y_, 0.0);
        nh_.param<double>("/uav_tf_offset_x", uav_tf_offset_x_, 0.0);
        nh_.param<double>("/uav_tf_offset_y", uav_tf_offset_y_, 0.0);

        double bound_width;
        resolution_ = simple_map_->map_->getResolution();
        nh_.param<double>("/bound_width", bound_width, 3.0);
        size_x_ = (unsigned int)((wx_max_ - wx_min_ + 2 * bound_width) / resolution_);
        size_y_ = (unsigned int)((wy_max_ - wy_min_ + 2 * bound_width) / resolution_);
        map_size_ = size_x_ * size_y_;
        ROS_WARN_STREAM("Map: " << size_x_ << " x " << size_y_);

        // Algorithm Params
        nh_.param<double>("utility_discount", util_discnt_, 0.7);
        nh_.param<double>("update_precision", update_precision_, 0.001);
        nh_.param<double>("uav_fronA_harmonic", uav_fronA_harmonic_, -1.0);
        nh_.param<double>("uav_fronB_harmonic", uav_fronB_harmonic_, -1.0);
        nh_.param<double>("uav_fronA_frontier", uav_fronA_frontier_, 1.0);
        nh_.param<double>("uav_fronB_frontier", uav_fronB_frontier_, 10.0);
        nh_.param<int>("relaxation_method", relaxation_method_, 2);
        nh_.param<double>("sor_w", sor_w_, 1.8);
        nh_.param<bool>("is_iter_bfs", is_iter_bfs_, true);

        if (relaxation_method_ != JACOBI_METHOD && relaxation_method_ != GS_METHOD && relaxation_method_ != SOR_METHOD){
            ROS_ERROR("Invalid Relaxation Method! Setting it to SOR");
            relaxation_method_ = SOR_METHOD;
        }

        // Flags
        is_rcv_uav_odom_ = is_rcv_ugv_odom_ = false;

        // ros message Container init
        rosMessageContainerInit();

        // data container
        charmap_no_inflation_   = std::vector<unsigned char>(map_size_);
        charmap_with_inflation_ = std::vector<unsigned char>(map_size_);
        harmonic_field_ugv_ = std::vector<double>(map_size_);
        charmap_info_ = std::vector<double>(map_size_);
    }

    bool DuetExploration::checkInBound(unsigned int idx) {
        unsigned int mx, my;
        double wx, wy;
        (simple_map_->map_)->indexToCells(idx, mx, my);
        (simple_map_->map_)->mapToWorld(mx, my, wx, wy);
        if (wx > wx_min_ && wx < wx_max_ && wy > wy_min_ && wy < wy_max_)
            return true;
        else
            return false;
    }

    bool DuetExploration::compFieldGradient(const std::vector<double> &field, std::vector<double> &gradient, unsigned int robot_type){
        std::vector<double> sobel_x(field.size(), 0.0);
        std::vector<double> sobel_y(field.size(), 0.0);
        gradient = std::vector<double>(field.size(), 0.0);

        // variables for visualization
        double viz_wx, viz_wy;
        unsigned int viz_mx, viz_my;
        geometry_msgs::Point child, parent;

        if (robot_type == TYPE_UGV){
            ugv_grad_viz_.header.stamp = ros::Time::now();
            ugv_grad_viz_.points.clear();
        }else if (robot_type == TYPE_UAV_FRONA){
            uav_grad_fronA_viz_.header.stamp = ros::Time::now();
            uav_grad_fronA_viz_.points.clear();
        }else if (robot_type == TYPE_UAV_FRONB){
            uav_grad_fronB_viz_.header.stamp = ros::Time::now();
            uav_grad_fronB_viz_.points.clear();
        }

        for (unsigned int idx = 0; idx < map_size_; idx++){
            if (charmap_with_inflation_[idx] != FREE_SPACE && robot_type == TYPE_UGV)
                continue; // only compute gradient for FREE cells
            if (idx % size_x_ == 0 || idx % size_x_ == size_x_ - 1 || idx < size_x_ || idx >= size_x_*(size_y_-1)){
                gradient[idx] = 0.0;
                continue; //TODO: neglect border cells, gradient set to 0
            }
            sobel_x[idx] = 2 * field[idx + 1] + field[idx - size_x_ + 1] + field[idx + size_x_ + 1]
                           - 2*field[idx - 1] - field[idx - size_x_ - 1] - field[idx + size_x_ - 1];
            sobel_y[idx] = 2 * field[idx + size_x_] + field[idx + size_x_ + 1] + field[idx + size_x_ - 1]
                           - 2*field[idx - size_x_] - field[idx - size_x_ + 1] - field[idx - size_x_ - 1];
            gradient[idx] = atan2(sobel_y[idx], sobel_x[idx]);

            // visualize gradient
            (simple_map_->map_)->indexToCells(idx, viz_mx, viz_my);
            (simple_map_->map_)->mapToWorld(viz_mx, viz_my, viz_wx, viz_wy);
            child.x = viz_wx;
            child.y = viz_wy;
            parent.x = child.x - 0.05 * cos(gradient[idx]);
            parent.y = child.y - 0.05 * sin(gradient[idx]);

            if (robot_type == TYPE_UGV){
                //ROS_INFO("add point!");
                ugv_grad_viz_.points.push_back(child);
                ugv_grad_viz_.points.push_back(parent);
            }else if (robot_type == TYPE_UAV_FRONA){
                uav_grad_fronA_viz_.points.push_back(child);
                uav_grad_fronA_viz_.points.push_back(parent);
            }else if (robot_type == TYPE_UAV_FRONB){
                uav_grad_fronB_viz_.points.push_back(child);
                uav_grad_fronB_viz_.points.push_back(parent);
            }
        }

        if (robot_type == TYPE_UGV){
            ugv_grad_viz_pub_.publish(ugv_grad_viz_);
        }else if (robot_type == TYPE_UAV_FRONA){
            uav_grad_fronA_viz_pub_.publish(uav_grad_fronA_viz_);
        }else if (robot_type == TYPE_UAV_FRONB){
            uav_grad_fronB_viz_pub_.publish(uav_grad_fronB_viz_);
        }

        return true;
    }

    void DuetExploration::copyMapsData()
    {
        map_inflation_->updateCosts(0, 0, size_x_, size_y_);
        for (unsigned int idx = 0; idx < size_x_ * size_y_; idx++)
        {
            charmap_no_inflation_[idx] = simple_map_->map_->getCost(idx);
            if (simple_map_->map_->getCostInflated(idx) == INSCRIBED_INFLATED_OBSTACLE)
                charmap_with_inflation_[idx] = LETHAL_OBSTACLE;
            else
                charmap_with_inflation_[idx] = simple_map_->map_->getCostInflated(idx);
        }
        simple_map_->pubMap();
    }

    double DuetExploration::countProgress(ofstream &time_file, ofstream &progress_file){

        unsigned int explored_cnt = 0, mx_min, mx_max, my_min, my_max, mx, my;
        simple_map_->map_->worldToMap(wx_min_, wy_min_, mx_min, my_min);
        simple_map_->map_->worldToMap(wx_max_, wy_max_, mx_max, my_max);
        int total_cnt = (mx_max - mx_min) * (my_max - my_min);
        for(unsigned int i = 0; i < charmap_no_inflation_.size(); i++){
            simple_map_->map_->indexToCells(i, mx, my);
            if (mx < mx_min || mx > mx_max || my < my_min || my > my_max)
                continue;
            if (charmap_no_inflation_[i] != NO_INFORMATION)
                explored_cnt++;
        }
        if (!charmap_no_inflation_.empty())
            ROS_INFO_STREAM("Exploration progress: " << (double(explored_cnt)) / total_cnt * 100 << "%");
        time_file << (ros::Time::now()-explore_start_time).toSec() << std::endl;
        progress_file << (double(explored_cnt)) / total_cnt * 100 << std::endl;

        return (double(explored_cnt)) / total_cnt;
    }

    void DuetExploration::getBfsSeq(){
        bfs_update_seq_.clear();
        // BFS
        std::vector<bool> visited_flag( map_size_, false);
        std::queue<unsigned int> bfs;

        for (auto it = new_ugv_boundary_idx_vec_.begin(); it != new_ugv_boundary_idx_vec_.end(); it++){ // preparation
            bfs.push(*it);
            visited_flag[*it] = true;
        }


        unsigned int front_idx;
        while (!bfs.empty()){ // start bfs now!
            front_idx = bfs.front();
            bfs.pop();

            BOOST_FOREACH(unsigned int nbr, harmonic_nhood4(front_idx, size_x_, size_y_)){
                if (charmap_info_[nbr] == FREE_VAL && !visited_flag[nbr]){
                    visited_flag[nbr] = true;
                    bfs_update_seq_.push_back(nbr);
                    bfs.push(nbr);
                }
            }
        }
    }

    bool DuetExploration::gradDescend(const std::vector<double> &gradient, unsigned int step_num, unsigned int robot_type)
    {
        unsigned int start_mx, start_my, iter_mx, iter_my, map_idx;
        double start_wx, start_wy, iter_wx, iter_wy, iter_grad, map_reso = (simple_map_->map_)->getResolution();
        if (robot_type == TYPE_UGV){
            start_wx = iter_wx = ugv_pose_.pose.position.x;
            start_wy = iter_wy = ugv_pose_.pose.position.y;
            ugv_path_viz_.header.stamp = ros::Time::now();
            ugv_path_viz_.points.clear();
        }else if (robot_type == TYPE_UAV_COMB){
            start_wx = iter_wx = uav_pose_.pose.position.x;
            start_wy = iter_wy = uav_pose_.pose.position.y;
            uav_path_viz_.header.stamp = ros::Time::now();
            uav_path_viz_.points.clear();
        }else{
            ROS_ERROR_STREAM("Invalid robot_type: " << robot_type);
            return false;
        }

        (simple_map_->map_)->worldToMap(start_wx, start_wy, start_mx, start_my);

        geometry_msgs::Point parent, child;

        for (unsigned int loop_idx = 0; loop_idx < step_num; loop_idx++){
            child.x = iter_wx;
            child.y = iter_wy;

            (simple_map_->map_)->worldToMap(iter_wx, iter_wy, iter_mx, iter_my);
            if ((simple_map_->map_)->getCost(iter_mx, iter_my) != FREE_SPACE)
                break;// path ends when out of free space
            map_idx = (simple_map_->map_)->getIndex(iter_mx, iter_my);
            iter_grad = gradient[map_idx];


            if (iter_grad >= PI/4 && iter_grad <= PI*3/4){//Y > X and Y > 0
                iter_wy -= map_reso;
                iter_wx -= map_reso / tan(iter_grad);
            }else if (iter_grad <= -PI/4 && iter_grad >= -PI*3/4){//Y > X and Y < 0
                iter_wy += map_reso;
                iter_wx += map_reso / tan(iter_grad);
            }else if (iter_grad <= PI/4 && iter_grad >= -PI/4){//X > Y and X > 0
                iter_wx -= map_reso;
                iter_wy -= map_reso * tan(iter_grad);
            }else if (iter_grad >= PI*3/4 || iter_grad <= -PI*3/4){ // X > Y and X < 0
                iter_wx += map_reso;
                iter_wy += map_reso * tan(iter_grad);
            }else{
                ROS_WARN_STREAM("Something WRONG: grad: " << iter_grad << " , in deg: " << iter_grad * 180 / PI);
            }

            parent.x = iter_wx;
            parent.y = iter_wy;
            //visualization

            if (robot_type == TYPE_UGV){
                child.z = parent.z = 0.0;
                ugv_path_viz_.points.push_back(child);
                ugv_path_viz_.points.push_back(parent);
            }else if (robot_type == TYPE_UAV_COMB){
                child.z = parent.z = 0.0;
                uav_path_viz_.points.push_back(child);
                uav_path_viz_.points.push_back(parent);
            }
        }

        if (robot_type == TYPE_UGV){
            ugv_path_viz_pub_.publish(ugv_path_viz_);
        }else if (robot_type == TYPE_UAV_COMB){
            uav_path_viz_pub_.publish(uav_path_viz_);
        }

        // ****************************************************************** //
        // start to generate a fucking Bezier safe trajectory 
        Path2D path2D;
        for(auto ptr:ugv_path_viz_.points)
        {   
            array<double, 2> pt = {ptr.x, ptr.y};
            path2D.push_back(pt);
        }
        if(path2D.size()==0) return false;
        
        /*cout<<"path2D size : "<<path2D.size()<<endl;
        cout<<"ugv_path_viz_.points size : "<<ugv_path_viz_.points.size()<<endl;*/

/*        for(auto ptr:ugv_path_viz_.points)
            cout<<ptr.x<<" , "<<ptr.y<<endl;*/
        ros::Time time1= ros::Time::now();
        vector<Cube2D> corridor = corridorGeneration(path2D);

        MatrixXd pos = MatrixXd::Zero(2,2);
        MatrixXd vel = MatrixXd::Zero(2,2);
        MatrixXd acc = MatrixXd::Zero(2,2);

        // TODO
        pos.row(0) = Vector2d(explorer_->ugv_pose_.pose.position.x, explorer_->ugv_pose_.pose.position.y);
        pos.row(1) = Vector2d(path2D.back()[0], path2D.back()[1]);

        vel.row(0) = Vector2d(explorer_->ugv_odom_.twist.twist.linear.x,  explorer_->ugv_odom_.twist.twist.linear.y);
        acc.row(0) = Vector2d(0,0);//explorer_->ugv_odom_.twist.twist.angular.x, explorer_->ugv_odom_.twist.twist.angular.y);
        //

        //timeAllocation(corridor);
        _Time=VectorXd::Zero(corridor.size());
        for(int i= 0; i<corridor.size(); i++) _Time(i)= corridor[i].t;
        visCorridor(corridor);

#if 1
        _segment_num = corridor.size();
        
        double obj;
        double _MAX_Vel = 1.0;
        double _MAX_Acc = 0.5; // now no use
        double _cube_margin = 0.0;

        _PolyCoeff = _trajectoryGenerator.BezierPloyCoeffGeneration2D(  
                     corridor, _MQM, _C, _Cv, _Ca, pos, vel, acc, _MAX_Vel, _MAX_Acc, _traj_order, _minimize_order, obj, _cube_margin );

        ROS_WARN("The objective of the program is %f", obj);
        //ROS_WARN("The time consumation of the program is %f", (time_aft_opt - time_bef_opt).toSec());

        if(_PolyCoeff.rows() == 3 && _PolyCoeff.cols() == 3){
            ROS_WARN("Cannot find a feasible and optimal solution, somthing wrong with the mosek solver ... ");
            traj_finish=0;
            ros::Time time2= ros::Time::now();
            btf<<(time2-time1).toSec()<<endl;
                return false;
              
        }
        else
        {
            ROS_WARN("Successfully found a optimal solution .");
            _traj = getBezierTraj();
            traj_finish=1;
            ros::Time time2= ros::Time::now();
            bts<<(time2-time1).toSec()<<endl;

            explorer_->ugv_traj_pub_.publish(_traj);
            ROS_WARN("After traj pub.");
            _traj_id ++;
            //cout<<"_Time:\n"<<_Time<<endl;
            visBezierTrajectory(_PolyCoeff, _Time);
        }
#endif
        return true;
    }

    bool DuetExploration::initCharmapInfo() {
        new_ugv_boundary_idx_vec_.clear();

        std::vector<double> last_charmap_info(charmap_info_);

        unsigned int iter_mx, iter_my;

        for(unsigned int idx = 0; idx < charmap_no_inflation_.size(); idx++){
            if(!checkInBound(idx)){
                charmap_info_[idx] = OUT_OF_BOUND_VAL;
            }
            else if (charmap_with_inflation_[idx] == LETHAL_OBSTACLE){
                if (last_charmap_info[idx] != OBSTACLE_VAL){
                    new_ugv_boundary_idx_vec_.push_back(idx); // push new obstacle boundary into vector
                    //unsigned int mx, my;
                    //simple_map_->map_->indexToCells(idx, mx, my);
                    //ROS_INFO_STREAM("New obs! Coords: " << mx <<", " << my << ", value: " << charmap_info_[idx]);
                    //charmap_info_[idx] = OBSTACLE_VAL;
                }
                charmap_info_[idx] = OBSTACLE_VAL;
            }
            else if(charmap_no_inflation_[idx] == FREE_SPACE){
                charmap_info_[idx] = FREE_VAL;
            }
            else if(charmap_no_inflation_[idx] == NO_INFORMATION){
                charmap_info_[idx] = UNKNOWN_VAL;
            }
        }

        // search for frontiers
        std::string frontier_travel_point = "closest";
        frontierA_list_.clear();// isn't necessary
        frontierA_cell_list_.clear();
        frontierB_cell_list_.clear();// isn't necessary
        FrontierSearch frontierSearch(*(simple_map_->map_), min_frontier_size_, frontier_travel_point,
                                      wx_min_, wx_max_, wy_min_, wy_max_);
        frontierA_list_ = frontierSearch.searchFrontierA(charmap_no_inflation_);
        frontierB_cell_list_ = frontierSearch.searchFrontierBCell(charmap_no_inflation_);
        if (frontierA_list_.empty() && frontierB_cell_list_.empty()){
            ROS_WARN("No frontiers found! Exploration complete!");
            return false;
        }
        unsigned int map_idx;
        duet_exploration::FrontierCell temp;
        BOOST_FOREACH(duet_exploration::Frontier frontier, frontierA_list_) {
            BOOST_FOREACH(geometry_msgs::Point frontier_cell_point, frontier.frontier_cells) {
                map_idx = (simple_map_->map_)->getIndex(frontier_cell_point.x, frontier_cell_point.y);

                if (charmap_info_[map_idx] == OBSTACLE_VAL)
                    continue;

                if (last_charmap_info[map_idx] != frontier.size){
                    new_ugv_boundary_idx_vec_.push_back(map_idx);
                }
                charmap_info_[map_idx] = frontier.size;

                temp.frontier_cell_point = frontier_cell_point;
                //temp.frontier_size = frontier.size; //don't need size in UAV
                temp.utility = 1.0;
                temp.map_idx = map_idx;
                frontierA_cell_list_.push_back(temp);
            }
        }

        // Since frontier B is not used in harmonic, we disgard it in charmap_info
        for(auto it = frontierB_cell_list_.begin(); it != frontierB_cell_list_.end(); it++){
            map_idx = (simple_map_->map_)->getIndex(it->frontier_cell_point.x, it->frontier_cell_point.y);
            charmap_info_[map_idx] = FRONTIER_B_VAL;
        }
        

        //ROS_INFO_STREAM("Boundary-Changed Cells: " << new_ugv_boundary_idx_vec_.size());

        if (new_ugv_boundary_idx_vec_.empty())
            ROS_WARN("No new boundary! Using original update-sequence");
        else
            ROS_INFO_STREAM("Boundary-Changed Cells: " << new_ugv_boundary_idx_vec_.size());
        return true;
    }

    void DuetExploration::initHarmonicFieldUGV()
    {
        if (harmonic_field_ugv_.size() != charmap_info_.size()){
            harmonic_field_ugv_ = std::vector<double>(charmap_info_.size());
            ROS_ERROR("Map size has changed!");
        }
        for(unsigned int idx = 0; idx < harmonic_field_ugv_.size(); idx++){

            if (charmap_info_[idx] == OUT_OF_BOUND_VAL){
                harmonic_field_ugv_[idx] = 0.0;
            }

            else if (charmap_info_[idx] > 0){
                harmonic_field_ugv_[idx] = 0.0 - charmap_info_[idx]; // linear to frontierA size, as a result linear to gradient
            }

            else if(charmap_info_[idx] == OBSTACLE_VAL){
                harmonic_field_ugv_[idx] = 0.0;
            }

            else{ // any other cases, see as obstacle
                harmonic_field_ugv_[idx] = 0.0;
            }
        }
        // push ugv_pose into harmonic_field
        unsigned int mx, my, ugv_idx;
        (simple_map_->map_)->worldToMap(ugv_pose_.pose.position.x, ugv_pose_.pose.position.y, mx, my);
        ugv_idx = (simple_map_->map_)->getIndex(mx, my);
        harmonic_field_ugv_[ugv_idx] = 0.0;
    }

    void DuetExploration::iterUpdate(std::vector<double> &harmonic_field, double &update_change, unsigned int robot_type) {
        // TODO: Note that code below doesn't compute border cells (just neglect them), since there should be no free cells on borders
        std::vector<double> field_last_iter; //container for field from last iteration
        for (auto it = harmonic_field.begin(); it != harmonic_field.end(); it++){
            field_last_iter.push_back(*it);
        }

        double update_change_avr = 0.0, update_change_max = 0.0;

        unsigned int free_cell_cnt = 0;
        for (unsigned int idx = 0; idx < harmonic_field.size(); idx++){

            if (charmap_info_[idx] == OUT_OF_BOUND_VAL)
                continue;

            if (charmap_info_[idx] != FREE_VAL && robot_type == TYPE_UGV){
                continue; //only update FREE_SPACE cells
            }

            free_cell_cnt ++;

            switch (relaxation_method_){
                case JACOBI_METHOD:
                    harmonic_field[idx] = 0.25 * (field_last_iter[idx - 1] + field_last_iter[idx + 1]
                                                  + field_last_iter[idx - size_x_] + field_last_iter[idx + size_x_]);
                    break;
                case GS_METHOD:
                    harmonic_field[idx] = 0.25 * (harmonic_field[idx - 1] + field_last_iter[idx + 1]
                                                  + harmonic_field[idx - size_x_] + field_last_iter[idx + size_x_]);
                    break;
                case SOR_METHOD:
                    harmonic_field[idx] = field_last_iter[idx]
                                          + 0.25 * sor_w_ * (harmonic_field[idx - 1] + field_last_iter[idx + 1]
                                                             + harmonic_field[idx - size_x_] + field_last_iter[idx + size_x_] - 4 * field_last_iter[idx]);
                    break;
                default:
                    break;
            }

            update_change_avr += fabs(harmonic_field[idx] - field_last_iter[idx]);
            if (update_change_max < fabs(harmonic_field[idx] - field_last_iter[idx]))
                update_change_max = fabs(harmonic_field[idx] - field_last_iter[idx]);
        }
        update_change_avr /= free_cell_cnt;

        update_change = update_change_max;
    }

    bool DuetExploration::iterUpdateBFS(std::vector<double> &harmonic_field, double &update_change) {
        std::vector<double> field_last_iter; //container for field from last iteration
        for (auto it = harmonic_field.begin(); it != harmonic_field.end(); it++){
            field_last_iter.push_back(*it);
        }

        double update_change_max = 0.0;

        for (auto it = bfs_update_seq_.begin(); it != bfs_update_seq_.end(); it++){
            switch (relaxation_method_){
                case JACOBI_METHOD:
                    harmonic_field[*it] = 0.25 * (field_last_iter[*it - 1] + field_last_iter[*it + 1]
                                          + field_last_iter[*it - size_x_] + field_last_iter[*it + size_x_]);
                    break;
                case GS_METHOD:
                    harmonic_field[*it] = 0.25 * (harmonic_field[*it - 1] + field_last_iter[*it + 1]
                                          + harmonic_field[*it - size_x_] + field_last_iter[*it + size_x_]);
                    break;
                case SOR_METHOD:
                    harmonic_field[*it] = harmonic_field[*it]
                                          + 0.25 * sor_w_ * (harmonic_field[*it - 1] + harmonic_field[*it + 1]
                                          + harmonic_field[*it - size_x_] + harmonic_field[*it + size_x_] - 4 * harmonic_field[*it]);
                    break;
                default:
                    break;
            }
            // compute update_change max
            if (update_change_max < fabs(harmonic_field[*it] - field_last_iter[*it]))
                update_change_max = fabs(harmonic_field[*it] - field_last_iter[*it]);
        }

        update_change = update_change_max;
    }

    bool DuetExploration::procPickPubUAVGoal() {
        duet_exploration::FrontierCell selected_cell;
        selected_cell.score = 0.0;
        unsigned int uav_mx, uav_my;
        (simple_map_->map_)->worldToMap(uav_pose_.pose.position.x, uav_pose_.pose.position.y, uav_mx, uav_my);

        double goal_angle, diff_angle, vel_angle = atan2(uav_vel_.linear.y, uav_vel_.linear.x);
        for (auto it = frontierA_cell_list_.begin(); it != frontierA_cell_list_.end(); it++){
            it->information_gain = uav_fronA_frontier_ * it->utility;
            // TODO: Need to do more about cost. Generate fixed-speed traj from current position to goal, considering current velocity(direction). Use LUT if time-consuming.
            it->traverse_cost_air = sqrt((it->frontier_cell_point.x - uav_mx)*(it->frontier_cell_point.x - uav_mx)
                                         + (it->frontier_cell_point.y - uav_my) * (it->frontier_cell_point.y - uav_my));
            // perform consistent bonus if goal is near.
            if (it->traverse_cost_air < 2 * UAV_fov_x_){
                goal_angle = atan2(it->frontier_cell_point.y - uav_my, it->frontier_cell_point.x - uav_mx);
                diff_angle = angle_diff(goal_angle, vel_angle);
                if (diff_angle < 4/PI)
                    it->traverse_cost_air /= 2 * cos(diff_angle);
            }

            it->score = it->information_gain / it->traverse_cost_air;
            if (it->score > selected_cell.score)
                selected_cell = *it;
        }

        for (auto it = frontierB_cell_list_.begin();it != frontierB_cell_list_.end(); it++){
            it->information_gain = uav_fronB_frontier_;
            it->traverse_cost_air = sqrt((it->frontier_cell_point.x - uav_mx)*(it->frontier_cell_point.x - uav_mx)
                                         + (it->frontier_cell_point.y - uav_my) * (it->frontier_cell_point.y - uav_my));
            // perform consistent bonus if goal is near.
            if (it->traverse_cost_air < 2 * UAV_fov_x_){
                goal_angle = atan2(it->frontier_cell_point.y - uav_my, it->frontier_cell_point.x - uav_mx);
                diff_angle = angle_diff(goal_angle, vel_angle);
                if (diff_angle < 4/PI)
                    it->traverse_cost_air /= 2 * cos(diff_angle);
            }

            it->score = it->information_gain / it->traverse_cost_air;
            if (it->score > selected_cell.score)
                selected_cell = *it;
        }

        // publish selected frontier cell
        double goal_wx, goal_wy;
        (simple_map_->map_)->mapToWorld(selected_cell.frontier_cell_point.x, selected_cell.frontier_cell_point.y, goal_wx, goal_wy);
        uav_goal_.header.stamp = ros::Time::now();
        uav_goal_.point.x = goal_wx;
        uav_goal_.point.y = goal_wy;
        uav_goal_pub_.publish(uav_goal_);
    }

    bool DuetExploration::motion_primitive(){
        ros::Time time1= ros::Time::now();
        Eigen::VectorXd uav_plan_pos= Eigen::VectorXd::Zero(4);
        uav_plan_pos(0)= uav_odom_.pose.pose.position.x;
        uav_plan_pos(1)= uav_odom_.twist.twist.linear.x;
        uav_plan_pos(2)= uav_odom_.pose.pose.position.y;
        uav_plan_pos(3)= uav_odom_.twist.twist.linear.y;
        path_search new_path(uav_plan_pos, uav_odom_.header.stamp.toSec());
        ros::Time time2= ros::Time::now();
        std::cout<<"Time consumed in motion primitive:"<<(time2-time1).toSec()<<std::endl;
        mp<<(time2-time1).toSec()<<"\n";

        return new_path.find_max_path();

    }

    bool DuetExploration::pubGrad(const std::vector<double> &gradient, ros::Publisher &pub) {
        duet_exploration::MultiArrayWithHeader mat;
        mat.header.frame_id = global_frame_name_;
        mat.header.stamp = ros::Time::now();
        mat.array.layout.data_offset = size_x_;
        mat.array.data.clear();
        for (auto it = gradient.begin(); it != gradient.end(); it++){
            mat.array.data.push_back(*it);
        }

        pub.publish(mat);
    }

    void DuetExploration::visCorridor(vector<Cube2D> corridor)
    {   
        for(auto & mk: cube_vis.markers) 
            mk.action = visualization_msgs::Marker::DELETE;
        
        corridor_vis_pub_.publish(cube_vis);

        cube_vis.markers.clear();

        visualization_msgs::Marker mk;
        mk.header.frame_id = "/robot_1/odom";
        mk.header.stamp = ros::Time::now();
        mk.ns = "corridor";
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;

        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;

        mk.color.a = 0.7;
        mk.color.r = 0.0;
        mk.color.g = 1.0;
        mk.color.b = 0.0;

        int idx = 0;
        for(int i = 0; i < int(corridor.size()); i++)
        {   
            mk.id = idx;

            mk.pose.position.x = (corridor[i].vertex(0, 0) + corridor[i].vertex(3, 0) ) / 2.0; 
            mk.pose.position.y = (corridor[i].vertex(0, 1) + corridor[i].vertex(1, 1) ) / 2.0; 
            mk.pose.position.z = -0.2;

            mk.scale.x = (corridor[i].vertex(0, 0) - corridor[i].vertex(3, 0) );
            mk.scale.y = (corridor[i].vertex(1, 1) - corridor[i].vertex(0, 1) );
            mk.scale.z = 0.1;

            idx ++;
            cube_vis.markers.push_back(mk);
        }

        corridor_vis_pub_.publish(cube_vis);
    }

    void DuetExploration::rosMessageContainerInit()
    {
        grid_map_no_inflation_.header.frame_id        = global_frame_name_;
        grid_map_no_inflation_.header.stamp           = ros::Time::now();
        grid_map_no_inflation_.info.origin.position.x = simple_map_->map_->getOriginX();
        grid_map_no_inflation_.info.origin.position.y = simple_map_->map_->getOriginY();
        grid_map_no_inflation_.info.resolution        = simple_map_->map_->getResolution();

        harmonic_field_ugv_viz_.header.frame_id        = global_frame_name_;
        harmonic_field_ugv_viz_.header.stamp           = ros::Time::now();
        harmonic_field_ugv_viz_.info.origin.position.x = simple_map_->map_->getOriginX();
        harmonic_field_ugv_viz_.info.origin.position.y = simple_map_->map_->getOriginY();
        harmonic_field_ugv_viz_.info.resolution        = simple_map_->map_->getResolution();

        harmonic_field_uav_fronA_viz_.header.frame_id        = global_frame_name_;
        harmonic_field_uav_fronA_viz_.header.stamp           = ros::Time::now();
        harmonic_field_uav_fronA_viz_.info.origin.position.x = simple_map_->map_->getOriginX();
        harmonic_field_uav_fronA_viz_.info.origin.position.y = simple_map_->map_->getOriginY();
        harmonic_field_uav_fronA_viz_.info.resolution        = simple_map_->map_->getResolution();

        ugv_path_viz_.header.frame_id = global_frame_name_;
        ugv_path_viz_.action = visualization_msgs::Marker::ADD;
        ugv_path_viz_.pose.orientation.w = 1.0;
        ugv_path_viz_.id = 0;
        ugv_path_viz_.type = visualization_msgs::Marker::LINE_LIST;
        ugv_path_viz_.scale.x = 0.05;
        ugv_path_viz_.color.b = ugv_path_viz_.color.a = 1.0;

        uav_path_viz_.header.frame_id = global_frame_name_;
        uav_path_viz_.action = visualization_msgs::Marker::ADD;
        uav_path_viz_.pose.orientation.w = 1.0;
        uav_path_viz_.id = 0;
        uav_path_viz_.type = visualization_msgs::Marker::LINE_LIST;
        uav_path_viz_.scale.x = 0.05;
        uav_path_viz_.color.g = uav_path_viz_.color.a = 1.0;

        ugv_grad_viz_.header.frame_id = global_frame_name_;
        ugv_grad_viz_.action = visualization_msgs::Marker::ADD;
        ugv_grad_viz_.pose.orientation.w = 1.0;
        ugv_grad_viz_.id = 0;
        ugv_grad_viz_.type = visualization_msgs::Marker::LINE_LIST;
        ugv_grad_viz_.scale.x = 0.02;
        ugv_grad_viz_.color.r = ugv_grad_viz_.color.a = 1.0;

        uav_grad_viz_.header.frame_id = global_frame_name_;
        uav_grad_viz_.action = visualization_msgs::Marker::ADD;
        uav_grad_viz_.pose.orientation.w = 1.0;
        uav_grad_viz_.id = 0;
        uav_grad_viz_.type = visualization_msgs::Marker::LINE_LIST;
        uav_grad_viz_.scale.x = 0.02;
        uav_grad_viz_.color.r = uav_grad_viz_.color.a = 1.0;

        uav_goal_.header.frame_id = global_frame_name_;
    }

    void DuetExploration::ugv_odom_call_back(const nav_msgs::Odometry ugv_odom_origin){
        //ROS_WARN("receive ugv odom");
        ugv_odom_ = ugv_odom_origin;
        ugv_pose_.pose   = ugv_odom_origin.pose.pose;
        ugv_pose_.pose.position.x -= ugv_tf_offset_x_;
        ugv_pose_.pose.position.y -= ugv_tf_offset_y_;
        ugv_pose_.header = ugv_odom_origin.header;

        is_rcv_ugv_odom_ = true;
    }

    void DuetExploration::uav_odom_call_back(const nav_msgs::Odometry &uav_odom_origin){
        //ROS_WARN("receive uav odom");
        uav_odom_ = uav_odom_origin;
        geometry_msgs::PoseStamped tmp_pose = uav_pose_;
        uav_pose_.pose   = uav_odom_origin.pose.pose;
        uav_pose_.pose.position.x -= uav_tf_offset_x_;
        uav_pose_.pose.position.y -= uav_tf_offset_y_;
        uav_pose_.header = uav_odom_origin.header;
        uav_vel_.linear.x = uav_odom_origin.twist.twist.linear.x;
        uav_vel_.linear.y = uav_odom_origin.twist.twist.linear.y;

        is_rcv_uav_odom_ = true;
    }
    bool DuetExploration::utilDiscount(){
        std::vector<bool> discounted(map_size_, false);
        unsigned int check_step = 5, check_total_step = 30, check_mx, check_my;
        double dist;
        if (check_total_step > ugv_path_idx_.size())
            check_total_step = ugv_path_idx_.size();
        for (unsigned int idx = check_step - 1; idx < check_total_step; idx += check_step){
            (simple_map_->map_)->indexToCells(ugv_path_idx_[idx], check_mx, check_my);
            for (auto it = frontierA_cell_list_.begin(); it != frontierA_cell_list_.end(); it++){
                dist = sqrt(pow(check_mx - it->frontier_cell_point.x, 2.0) + pow(check_my - it->frontier_cell_point.y, 2.0)) / resolution_;
                // perform utility discount if: not discounted before, distance < range, and raytrace free
                if (!discounted[idx] && dist < ground_range_ && simple_map_->map_->checkRaytraceFree(check_mx, check_my, it->frontier_cell_point.x, it->frontier_cell_point.y)){
                    discounted[it->map_idx] = true;
                    it->utility *= util_discnt_;
                }
            }
        }
    }

    void DuetExploration::visualize_field(const std::vector<double> &field, nav_msgs::OccupancyGrid &grid_viz, ros::Publisher &pub)
    {
        grid_viz.info.width  = size_x_;
        grid_viz.info.height = size_y_;

        grid_viz.data.clear();
        double min = 0.0;
        for (unsigned int idx = 0; idx < map_size_; idx++){
            if (field[idx] < min)
                min = field[idx];
        }
        int tmp_val;
        for (unsigned int idx = 0; idx < map_size_; idx++){
            tmp_val = int(field[idx] / min * 100.0);
            grid_viz.data.push_back(tmp_val);
        }
        pub.publish(grid_viz);
    }

    bool DuetExploration::visualize_frontier(){
        //pointcloud for visualization purposes
        pcl::PointCloud<pcl::PointXYZI> frontier_cell_viz;
        pcl::PointXYZI frontier_point_viz(50);//initialize with 50 intensi      ty
        pcl::PointXYZI frontier_point_viz_B(30);//initialize viz for frontierB with 30 intensity
        double viz_wx, viz_wy;
        //ROS_INFO_STREAM("Size of frontierA list: " << frontierA_cell_list.size());
        BOOST_FOREACH(FrontierCell frontier_cell, frontierA_cell_list_) {
            //load frontier into visualization poitncloud
            (simple_map_->map_)->mapToWorld(
                    (unsigned int) frontier_cell.frontier_cell_point.x,
                    (unsigned int) frontier_cell.frontier_cell_point.y,
                    viz_wx, viz_wy);
            frontier_point_viz.x = viz_wx;
            frontier_point_viz.y = viz_wy;
            frontier_cell_viz.push_back(frontier_point_viz);
        }
        //ROS_INFO_STREAM("Size of frontierB list: " << frontierB_cell_list.size());
        BOOST_FOREACH(FrontierCell frontier_cell, frontierB_cell_list_) {
            //load frontier into visualization poitncloud
            (simple_map_->map_)->mapToWorld(
                    (unsigned int) frontier_cell.frontier_cell_point.x,
                    (unsigned int) frontier_cell.frontier_cell_point.y,
                    viz_wx, viz_wy);
            frontier_point_viz_B.x = viz_wx;
            frontier_point_viz_B.y = viz_wy;
            frontier_point_viz_B.z = 0.8;
            frontier_cell_viz.push_back(frontier_point_viz_B);
        }

        //publish visualization point cloud
        sensor_msgs::PointCloud2 frontier_cell_viz_output;
        pcl::toROSMsg(frontier_cell_viz, frontier_cell_viz_output);
        frontier_cell_viz_output.header.frame_id = global_frame_name_;
        frontier_cell_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub_.publish(frontier_cell_viz_output);

        //ROS_INFO("Time flag viz");
    }

    bool DuetExploration::compAirGoalInfoGain(double wx, double wy, double& info_gain)
    {
        unsigned mx, my;
        if(!((simple_map_->map_)->worldToMap(wx, wy, mx, my))) return false;

        int left_mx, right_mx, top_my, bottom_my;
        unsigned int iter_x, iter_y;

        left_mx  = mx - UAV_fov_x_/(2*resolution_);
        right_mx = mx + UAV_fov_x_/(2*resolution_);
        bottom_my= my - UAV_fov_y_/(2*resolution_);
        top_my   = my + UAV_fov_y_/(2*resolution_);

        left_mx  = left_mx   > 0       ? left_mx   : 0;
        right_mx = right_mx  < size_x_ ? right_mx  : size_x_ - 1;
        bottom_my= bottom_my > 0       ? bottom_my : 0;
        top_my   = top_my    < size_y_ ? top_my    : size_y_ - 1;

        info_gain = 0.0;
        unsigned int vis_fronB_cnt = 0, vis_fron_cnt = 0;
        for (iter_y = bottom_my; iter_y <= top_my; iter_y++){

            for (iter_x = left_mx; iter_x <= right_mx; iter_x++){

                unsigned int idx = (simple_map_->map_)->getIndex(iter_x, iter_y); // get charMap index

                if (charmap_info_[idx] > 0){
                    vis_fron_cnt++;
                    info_gain += INFO_FRONA;
                }else if(charmap_info_[idx] == FRONTIER_B_VAL){
                    vis_fronB_cnt ++;
                    vis_fron_cnt  ++;
                    info_gain += INFO_FRONB;
                }else if(charmap_info_[idx] == UNKNOWN_VAL){
                    info_gain += INFO_UNKNWON;
                }
            }
        }
        if (vis_fron_cnt <= 3) // discard goal candidate without frontier visibility
        {
            info_gain = 0.0;
            //return false;
        }

        if (vis_fronB_cnt == 0 && !frontierB_cell_list_.empty()){ // discard goal candidate without frontierB visibility if present
            info_gain = 0.0;
            //return false;
        }

        return true;
        
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "harmonic_duet_node");
    
    duet_exploration::DuetExploration explorer("harmonic_duet_node", "/robot_1/odom");   
    explorer_ = &explorer;
    _start_time = ros::Time::now();

    Bernstein _bernstein;
    if(_bernstein.setParam(3, 12, _minimize_order) == -1)
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set "); 

    _MQM = _bernstein.getMQM()[_traj_order];
    _C   = _bernstein.getC()[_traj_order];
    _Cv  = _bernstein.getC_v()[_traj_order];
    _Ca  = _bernstein.getC_a()[_traj_order];
    _Cj  = _bernstein.getC_j()[_traj_order];

    mp.open("/home/gf/Documents/explore_progress_log/motion_primitive_solve_time.txt");
    bts.open("/home/gf/Documents/explore_progress_log/beizer_trajectory_solve_time.txt");
    btf.open("/home/gf/Documents/explore_progress_log/beizer_trajectory_fail_time.txt");
    explore_start_time=ros::Time::now();
    explorer.explore_main_loop();
}
