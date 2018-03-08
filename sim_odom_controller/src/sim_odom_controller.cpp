#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sys/time.h>
#include <string>
#include <time.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <frontier_exploration/MultiArrayWithHeader.h>
#include <quadrotor_msgs/PositionCommand.h>

#define ugv_max_speed_x 0.5
#define ugv_max_speed_y 0.25
#define ugv_max_omega 0.5

#define uav_max_speed 1
#define uav_height 1.2


namespace sim_odom_controller{

    using namespace std;
    using namespace tf;
    using namespace Eigen;

    class SimOdomController{
    public:

        SimOdomController() : ugv_normal_(0), ugv_initial_yaw_(0), ugv_initial_omega_(0), kp_(10), kd_(1), nh_("~"), ugv_odom_frame_("/robot_1/base_footprint")
        {
            uav_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("uav_target",5);
            ugv_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("ugv_target",5);
			pub_position_cmd = nh_.advertise< quadrotor_msgs::PositionCommand >( "/position_cmd", 10 );


            uav_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("uav_cmd_vel",5);
            ugv_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("ugv_cmd_vel",5);

			sub_trigger = nh_.subscribe( "/traj_start_trigger", 100, &SimOdomController::trigger_callback, this);

            uav_coef_sub_ = nh_.subscribe("uav_coef", 1, &SimOdomController::uav_coef_call_back, this);
            ugv_coef_sub_ = nh_.subscribe("ugv_coef", 1, &SimOdomController::ugv_coef_call_back, this);

            uav_pose_sub_ = nh_.subscribe("uav_pose", 1, &SimOdomController::uav_pose_call_back, this);
            ugv_pose_sub_ = nh_.subscribe("ugv_pose", 10, &SimOdomController::ugv_pose_call_back, this);

            nh_.param<std::string>("global_frame", global_frame_, "/robot_1/map");

	    nh_.param<bool>("flag_is_sim", flag_is_sim_, true);

            nh_.param<bool>("/simulation", simulation_, true);

			traj_id=0;
			is_init= false;
			is_traj= false;

            ros::Duration(3).sleep();

        }


    private:

        bool ugv_normal_, simulation_;
        MatrixXd ugv_coef_;
        VectorXd ugv_beta_;
        VectorXd ugv_t_;
        double ugv_plan_time;

	bool flag_is_sim_;

        double ugv_initial_yaw_;
        double ugv_initial_omega_;

        MatrixXd uav_coef_;
        VectorXd uav_t_;

        double uav_plan_time;

        double kp_, kd_;

		 int traj_id;
        bool is_init ;
        bool is_traj ;

        std::string global_frame_, uav_odom_frame_, ugv_odom_frame_;

        ros::NodeHandle nh_;

        ros::Publisher uav_odom_pub_, ugv_odom_pub_, uav_cmd_vel_pub_, ugv_cmd_vel_pub_, pub_position_cmd;
        ros::Subscriber uav_coef_sub_, ugv_coef_sub_, uav_pose_sub_, ugv_pose_sub_,sub_trigger;

        tf::TransformListener tf_listener_;
        nav_msgs::Odometry uav_odom_des_, ugv_odom_des_;

        bool transformOdom(nav_msgs::Odometry &odom, std::string twist_frame){
            if (odom.header.frame_id != global_frame_){

                geometry_msgs::PoseStamped origin_pose, out_pose;
                origin_pose.header = odom.header;
                origin_pose.pose = odom.pose.pose;

                geometry_msgs::Vector3Stamped origin_v_l, origin_v_a, out_v_l, out_v_a;

                origin_v_l.header = origin_v_a.header = odom.header;
                if (!twist_frame.empty() && simulation_){
                    origin_v_a.header.frame_id = twist_frame;
                    origin_v_l.header.frame_id = twist_frame;
                }

                origin_v_a.vector = odom.twist.twist.angular;
                origin_v_l.vector = odom.twist.twist.linear;

                if (!tf_listener_.waitForTransform(global_frame_, odom.header.frame_id, odom.header.stamp, ros::Duration(5))){
                    ROS_ERROR_STREAM("Couldn't transform from "<<global_frame_<<" to "<< odom.header.frame_id);
                    return false;
                }

                while (true)
                {
                    try{
                        tf_listener_.transformPose(global_frame_, origin_pose, out_pose);

                        if (simulation_ && !twist_frame.empty()){
                            tf_listener_.transformVector("/robot_1/odom", origin_v_a, out_v_a);
                            tf_listener_.transformVector("/robot_1/odom", origin_v_l, out_v_l);
                        }else{
                            tf_listener_.transformVector(global_frame_, origin_v_a, out_v_a);
                            tf_listener_.transformVector(global_frame_, origin_v_l, out_v_l);
                        }
                        break;
                    }
                    catch (exception& e)
                    {
                        cout << e.what() << endl;
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

        void ugv_coef_call_back(const frontier_exploration::MultiArrayWithHeader &msg){
            ROS_INFO("Received UGV Coef!");

            if (msg.header.frame_id == "failsafe"){
                ROS_WARN("Failsafe encountered!");
                ugv_normal_ = false;
                //ToDo : should handle this and stop publishing cmd_vel
            }
            else{
            	ugv_plan_time=msg.header.stamp.toSec();
                ugv_normal_ = true;
                ugv_coef_.resize(msg.array.layout.dim[0].size,msg.array.layout.dim[1].size-4);
                MatrixXd e = MatrixXd::Zero(msg.array.layout.dim[0].size,msg.array.layout.dim[1].size);
                int ii = 0;
                for (int i = 0; i < e.rows(); ++i)
                    for (int j = 0; j < e.cols(); ++j)
                        e(i,j) = msg.array.data[ii++];


                ugv_coef_ = e.block(0, 0, e.rows(), e.cols() - 4);
                ugv_beta_ = e.col(ugv_coef_.cols() );
                ugv_t_    = e.col(ugv_coef_.cols() +1);
                ugv_initial_yaw_=e(0,ugv_coef_.cols() + 2);
                ugv_initial_omega_=e(0,ugv_coef_.cols() +3);
            }

        }

		void trigger_callback( const geometry_msgs::PoseStamped::ConstPtr& trigger_msg )
        {
            if ( is_init )
            {
                std::cout << "[#INFO] get traj trigger info." << std::endl;
                traj_id    = traj_id+1;//trigger_msg->header.seq + 1;
                is_traj    = true;
            }
        }

        void uav_coef_call_back(const frontier_exploration::MultiArrayWithHeader &msg){
            ROS_INFO("Received UAV Coef!");
            uav_coef_.resize(msg.array.layout.dim[0].size,msg.array.layout.dim[1].size-2);
            MatrixXd e = MatrixXd::Zero(msg.array.layout.dim[0].size,msg.array.layout.dim[1].size);
            //cout<<e.cols()<<endl;
            int ii = 0;
            for (int i = 0; i < e.rows(); ++i)
                for (int j = 0; j < e.cols(); ++j)
                    e(i, j) = msg.array.data[ii++];
            uav_coef_ = e.block(0,0,e.rows(),e.cols()-2);
            uav_plan_time=msg.header.stamp.toSec();
            uav_t_ = e.col(uav_coef_.cols()+1); // is this really ugv_coef?
			traj_id++;

        }

        void ugv_pose_call_back(const nav_msgs::Odometry &msg){
            ROS_INFO("Received UGV Odom!");

            nav_msgs::Odometry m = msg;
            transformOdom(m, ugv_odom_frame_);
            if(ugv_normal_){
                cout<<"ugv_t:\n"<<ugv_t_<<endl;
                double dT = m.header.stamp.toSec();
                double des_x = 0;
                double des_y = 0;
                double des_yaw = ugv_initial_yaw_;

                double des_vx = 0;
                double des_vy = 0;
                double des_omega = ugv_initial_omega_;

                for(int i=0; i<ugv_t_.size();i++){
                    if(dT<=ugv_t_(i)){
                        double tt =i>0? dT-ugv_t_(i-1) : dT-ugv_plan_time;
			switch(i){
				case 0: //if(msg.twist.twist.linear.x+msg.twist.twist.linear.y<0.4) 
					tt+=0.2;
					//else tt+=2*(msg.twist.twist.linear.x+msg.twist.twist.linear.y); 
					break;
				case 1: //if(msg.twist.twist.linear.x+msg.twist.twist.linear.y<0.4) 
					tt+=0.1;
                                        //else tt+=1.3*(msg.twist.twist.linear.x+msg.twist.twist.linear.y);
                                        break;

				case 2:  //if(msg.twist.twist.linear.x+msg.twist.twist.linear.y<0.4) 
					tt+=0.0;
                                        //else tt+=0.5*(msg.twist.twist.linear.x+msg.twist.twist.linear.y);
                                        break;

				default: break;

			}
                        //std::cout<<tt<<'\n';
                        //std::cout<<"T(i-1): "<<T(i-1)<<'\n';

                        Matrix<double, 1, 6> t_p;
                        t_p<< 1, tt, pow(tt,2), pow(tt,3), pow(tt,4), pow(tt,5);
                        Matrix<double, 1, 6> t_v;
                        t_v<< 0, 1, 2*tt, 3*pow(tt,2), 4*pow(tt,3),5*pow(tt,4);

                        VectorXd coef_x;
                        VectorXd coef_y;
                        coef_x = (ugv_coef_.block(i,0,1,6)).transpose();
                        coef_y = (ugv_coef_.block(i,6,1,6)).transpose();

                        des_x = t_p * coef_x;
                        des_y = t_p * coef_y;
                        des_vx = t_v * coef_x;
                        des_vy = t_v * coef_y;

                        des_yaw += ugv_initial_omega_ * tt + 0.5 * ugv_beta_(i) * tt * tt;
                        des_omega += ugv_beta_(i)*tt;
			ROS_WARN("Got UGV target");
                        break;
                    }
                    else{
                    	if(i==ugv_t_.size()-1){dT=ugv_t_(i); --i; continue;};
                    	if(i>0){
                    		double time_step=ugv_t_(i)-ugv_t_(i-1);
                        	des_yaw+=des_omega * time_step+0.5*ugv_beta_(i)*time_step*time_step;
                        	des_omega+=ugv_beta_(i)*time_step;
                    	}
                    	else{
                    		double time_step=ugv_t_(i)-ugv_plan_time;
                    	}
                    }
                }


                Quaterniond ori(m.pose.pose.orientation.w,  m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z);
                Matrix3d Rgi = ori.toRotationMatrix();
                double phi = asin(Rgi(2,1));
                double yaw = atan2(-Rgi(0,1)/cos(phi),Rgi(1,1)/cos(phi));

                geometry_msgs::Twist cmd_vel;
                double control_linear_gx = des_vx;//kp_ * (des_x-m.pose.pose.position.x) + kd_*(des_vx-m.twist.twist.linear.x);
                double control_linear_gy = des_vy;//kp_ * (des_y-m.pose.pose.position.y) + kd_*(des_vy-m.twist.twist.linear.y);



                cmd_vel.linear.x = control_linear_gx * cos(yaw) + control_linear_gy * sin(yaw);
                cmd_vel.linear.y = control_linear_gy * cos(yaw) - control_linear_gx * sin(yaw);
                cmd_vel.linear.z = 0;

                cmd_vel.angular.x=0;
                cmd_vel.angular.y=0;
                cmd_vel.angular.z = des_omega;//kp_ * (des_yaw-yaw)+kd_*(des_omega-m.twist.twist.angular.z);

                if(cmd_vel.linear.x>ugv_max_speed_x){
                	cmd_vel.linear.y*=ugv_max_speed_x/cmd_vel.linear.x;
                	cmd_vel.angular.z*=ugv_max_speed_x/cmd_vel.linear.x;
                	cmd_vel.linear.x=ugv_max_speed_x;
                }
                if(cmd_vel.linear.y>ugv_max_speed_y){
                	cmd_vel.linear.x*=ugv_max_speed_y/cmd_vel.linear.y;
                	cmd_vel.angular.z*=ugv_max_speed_y/cmd_vel.linear.y;
                	cmd_vel.linear.y=ugv_max_speed_y;
                }

                if(cmd_vel.angular.z>ugv_max_omega){
                	cmd_vel.linear.x*=ugv_max_omega/cmd_vel.angular.z;
                	cmd_vel.linear.y*=ugv_max_omega/cmd_vel.angular.z;
                	cmd_vel.angular.z=ugv_max_omega;
                }


                //std::cout<<"des_vx:"<<cmd_vel.linear.x <<"\n"<<"des_vy:"<<cmd_vel.linear.y<<"\n";


                

                ugv_cmd_vel_pub_.publish(cmd_vel);

                //ToDo publish odometry
                nav_msgs::Odometry des_pos;

                ori=AngleAxisd(des_yaw,Vector3d::UnitZ());
                des_pos.pose.pose.orientation.w=ori.w();
                des_pos.pose.pose.orientation.x=ori.x();
                des_pos.pose.pose.orientation.y=ori.y();
                des_pos.pose.pose.orientation.z=ori.z();

                des_pos.pose.pose.position.x=des_x;
                des_pos.pose.pose.position.y=des_y;
                des_pos.pose.pose.position.z=m.pose.pose.position.z;

                des_pos.twist.twist.linear.x=des_vx;
                des_pos.twist.twist.linear.y=des_vy;
                des_pos.twist.twist.linear.z=0;

                des_pos.twist.twist.angular.x=0;
                des_pos.twist.twist.angular.y=0;
                des_pos.twist.twist.angular.z=des_omega;
                des_pos.header.frame_id = "robot_1/odom";
                des_pos.header.stamp = ros::Time::now();
                ugv_odom_pub_.publish(des_pos);
		ROS_ERROR("");

            }
        }

        void uav_pose_call_back(const nav_msgs::Odometry &msg) {
            ROS_INFO("Received UAV Odom!");

            nav_msgs::Odometry m = msg;
            transformOdom(m, "");
            if (is_init) {
                if(is_traj) {
                    double dT = m.header.stamp.toSec();
                    double des_x = 0;
                    double des_y = 0;
                    double des_vx = 0;
                    double des_vy = 0;
                    bool traj_ok=false;
                    //cout<<uav_t_.size()<<endl;
                    for (int i = 0; i < uav_t_.size(); i++) {
                        //cout<<i<<endl;
                        if (dT < uav_t_(i)) {
                            double tt = i ? dT - uav_t_(i - 1) : dT - uav_plan_time;
                            //std::cout<<tt<<'\n';
                            //std::cout<<"T(i-1): "<<T(i-1)<<'\n';
                            //cout<<"a"<<endl;
                            Matrix<double, 1, 6> t_p;
                            t_p << 1, tt, pow(tt, 2), pow(tt, 3), pow(tt, 4), pow(tt, 5);
                            Matrix<double, 1, 6> t_v;
                            t_v << 0, 1, 2 * tt, 3 * pow(tt, 2), 4 * pow(tt, 3), 5 * pow(tt, 4);
                            //std::cout<<"T(i-1): "<<T(i-1)<<'\n';
                            //cout<<"b"<<endl;
                            VectorXd coef_x;
                            VectorXd coef_y;
                            cout << i << endl;
                            coef_x = (uav_coef_.block(i, 0, 1, 6)).transpose();
                            coef_y = (uav_coef_.block(i, 6, 1, 6)).transpose();
                            //std::cout<<"T(i-1): "<<T(i-1)<<'\n';
                           // cout << "c" << endl;
                            cout << t_p << endl;
                            cout << coef_x << endl;
                            des_x = t_p * coef_x;
                            des_y = t_p * coef_y;
                            des_vx = t_v * coef_x;
                            des_vy = t_v * coef_y;
                            traj_ok= true;
                            break;
                        }
                    }
                    quadrotor_msgs::PositionCommand position_cmd;
                    position_cmd.header.stamp    = m.header.stamp;
                    position_cmd.header.frame_id = "world";
                    if(traj_ok){
                        Quaterniond ori(m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                                    m.pose.pose.orientation.z);
                        Matrix3d Rgi = ori.toRotationMatrix();
                    //cout<<"e"<<endl;
                    double phi = asin(Rgi(2, 1));
                    double yaw = atan2(-Rgi(0, 1) / cos(phi), Rgi(1, 1) / cos(phi));
                    //cout<<"f"<<endl;
                    geometry_msgs::Twist cmd_vel;
                    double control_linear_gx = des_vx;//kp_ * (des_x-m.pose.pose.position.x) + kd_ * (des_vx-m.twist.twist.linear.x);
                    double control_linear_gy = des_vy;//kp_ * (des_y-m.pose.pose.position.y) + kd_ * (des_vy-m.twist.twist.linear.y);
                    cmd_vel.linear.x = des_vx;//control_linear_gx * cos(yaw) + control_linear_gy * sin(yaw);
                    cmd_vel.linear.y = des_vy;//control_linear_gy * cos(yaw) - control_linear_gx * sin(yaw);
                    cmd_vel.linear.z = 0;


                    cmd_vel.angular.x = 0;
                    cmd_vel.angular.y = 0;
                    cmd_vel.angular.z = 0;

                    uav_cmd_vel_pub_.publish(cmd_vel);

                    nav_msgs::Odometry des_pos;

                    des_pos.pose.pose.orientation.w = 1;
                    des_pos.pose.pose.orientation.x = 0;
                    des_pos.pose.pose.orientation.y = 0;
                    des_pos.pose.pose.orientation.z = 0;

                    des_pos.pose.pose.position.x = des_x;
                    des_pos.pose.pose.position.y = des_y;
                    des_pos.pose.pose.position.z = m.pose.pose.position.z;

                    des_pos.twist.twist.linear.x = des_vx;
                    des_pos.twist.twist.linear.y = des_vy;
                    des_pos.twist.twist.linear.z = 0;

                    des_pos.twist.twist.angular.x = 0;
                    des_pos.twist.twist.angular.y = 0;
                    des_pos.twist.twist.angular.z = 0;
                    uav_odom_pub_.publish(des_pos);



                        position_cmd.position.x      = des_x;
                        position_cmd.position.y      = des_y;
                        position_cmd.position.z      = uav_height;
                        position_cmd.velocity.x      = des_vx;
                        position_cmd.velocity.y      = des_vy;
                        position_cmd.velocity.z      = 0;
                        position_cmd.acceleration.x  = 0;
                        position_cmd.acceleration.y  = 0;
                        position_cmd.acceleration.z  = 0;
                        position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
                        position_cmd.trajectory_id = traj_id;
                    }
                    else
                    {
                        position_cmd.position.x      = des_x;
                        position_cmd.position.y      = des_y;
                        position_cmd.position.z      = m.pose.pose.position.z;
                        position_cmd.velocity.x      = 0.0;
                        position_cmd.velocity.y      = 0.0;
                        position_cmd.velocity.z      = 0.0;
                        position_cmd.acceleration.x  = 0.0;
                        position_cmd.acceleration.y  = 0.0;
                        position_cmd.acceleration.z  = 0.0;
                        position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
                        position_cmd.trajectory_id   = traj_id;
                    }
                    pub_position_cmd.publish( position_cmd );
                    //cout<<"h"<<endl;
                }
            }
            else
            {
                is_init = true;
            }
        }


    };

}



int main(int argc, char *argv[]){

	ros::init(argc, argv, "sim_odom_controller");

    sim_odom_controller::SimOdomController controller;

    ros::spin();
}
