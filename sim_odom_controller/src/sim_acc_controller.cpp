#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sys/time.h>
#include <string>
#include <time.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <frontier_exploration/MultiArrayWithHeader.h>

#define ugv_max_speed_x 0.5
#define ugv_max_speed_y 0.25
#define ugv_max_omega 0.5


namespace sim_odom_controller
{

    class SimAccController{

    public:
        SimAccController() : nh_("~"), init_flag_(0), flag_rec_ugv_acc_(false)
        {
            uav_odom_sub_ = nh_.subscribe("uav_odom", 1, &SimAccController::uav_odom_cb, this);
            ugv_odom_sub_ = nh_.subscribe("/ekf_node/odom_ekf", 1, &SimAccController::ugv_odom_cb, this);

            uav_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("uav_cmd_vel", 1);
            ugv_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);

            uav_acc_sub_ = nh_.subscribe("uav_acc", 1, &SimAccController::uav_acc_cb, this);
            ugv_acc_sub_ = nh_.subscribe("/ugv_acc", 1, &SimAccController::ugv_acc_cb, this);

            while (init_flag_ != 3){
                ros::spinOnce();
                ROS_WARN_STREAM("No acc!" << init_flag_);
                ros::Duration(1).sleep();
            }

        }

        void loop(){
            ros::Rate r(100);
            double dt_uav, dt_ugv;
            unsigned int cnt = 0;

            while (init_flag_ == 3){

                if (++cnt >= 10){
                    cnt = 0;
                    ros::spinOnce();
                    //ROS_WARN("spinflag");
                }

                ros::Time current_t = ros::Time::now();
                dt_uav = current_t.toSec() - uav_twist_.header.stamp.toSec();
                dt_ugv = current_t.toSec() - ugv_twist_.header.stamp.toSec();

                uav_cmd_container_.linear.x = uav_acc_.linear.x * dt_uav + uav_twist_.twist.linear.x;
                uav_cmd_container_.linear.y = uav_acc_.linear.y * dt_uav + uav_twist_.twist.linear.y;

                ugv_cmd_container_.linear.x = ugv_acc_.linear.x * dt_ugv + ugv_twist_.twist.linear.x;
                ugv_cmd_container_.linear.y = ugv_acc_.linear.y * dt_ugv + ugv_twist_.twist.linear.y;
                ugv_cmd_container_.angular.z= ugv_acc_.angular.z* dt_ugv + ugv_twist_.twist.angular.z;
		

		if(abs(ugv_cmd_container_.linear.x) > ugv_max_speed_x){
                        ugv_cmd_container_.linear.y*=ugv_max_speed_x/abs(ugv_cmd_container_.linear.x);
                        ugv_cmd_container_.angular.z*=ugv_max_speed_x/abs(ugv_cmd_container_.linear.x);
                        ugv_cmd_container_.linear.x *=ugv_max_speed_x/abs(ugv_cmd_container_.linear.x);
                }
                if(abs(ugv_cmd_container_.linear.y) > ugv_max_speed_y){
                        ugv_cmd_container_.linear.x*=ugv_max_speed_y/abs(ugv_cmd_container_.linear.y);
                        ugv_cmd_container_.angular.z*=ugv_max_speed_y/abs(ugv_cmd_container_.linear.y);
                        ugv_cmd_container_.linear.y*=ugv_max_speed_y/abs(ugv_cmd_container_.linear.y);
                }

                if(abs(ugv_cmd_container_.angular.z) >ugv_max_omega){
                        ugv_cmd_container_.linear.x*=ugv_max_omega/abs(ugv_cmd_container_.angular.z);
                        ugv_cmd_container_.linear.y*=ugv_max_omega/abs(ugv_cmd_container_.angular.z);
                        ugv_cmd_container_.angular.z*=ugv_max_omega/abs(ugv_cmd_container_.angular.z);
                }

                uav_cmd_pub_.publish(uav_cmd_container_);
                ugv_cmd_pub_.publish(ugv_cmd_container_);

                //ROS_WARN("flag");
                std::cout << "ori vx: " << ugv_twist_.twist.linear.x << ", des vx: " << ugv_cmd_container_.linear.x << ", acc: " << ugv_acc_.linear.x << ", dT: " << dt_ugv << std::endl;

                r.sleep();
            }

        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber uav_odom_sub_, ugv_odom_sub_;
        ros::Subscriber uav_acc_sub_,  ugv_acc_sub_;
        ros::Publisher  uav_cmd_pub_,  ugv_cmd_pub_;

        geometry_msgs::Twist uav_acc_, ugv_acc_, uav_cmd_container_, ugv_cmd_container_;
        geometry_msgs::TwistStamped uav_twist_, ugv_twist_;

        unsigned int init_flag_;

	bool flag_rec_ugv_acc_;

        void uav_odom_cb(const nav_msgs::Odometry &uav_odom_rec){
            ROS_INFO("Got UAV Odom!");

            uav_twist_.twist = uav_odom_rec.twist.twist;
            uav_twist_.header.stamp = uav_odom_rec.header.stamp;
        }
        void ugv_odom_cb(const nav_msgs::Odometry &ugv_odom_rec){

            if (flag_rec_ugv_acc_){

                ugv_twist_.twist = ugv_odom_rec.twist.twist;
                ugv_twist_.header.stamp = ugv_odom_rec.header.stamp;
                flag_rec_ugv_acc_ = false;
                ROS_INFO("UPDATE UGV Odom!");

            }
        }

        void uav_acc_cb(const geometry_msgs::Twist &uav_acc){
            ROS_ERROR("Got acc!");
            if (!(init_flag_ % 2)){
                init_flag_ += 1;
                ROS_INFO("Got UAV Acc!");
            }

            uav_acc_ = uav_acc;
        }

        void ugv_acc_cb(const geometry_msgs::Twist &ugv_acc){
            ROS_ERROR("Got acc!");
            if (!(init_flag_ / 2)){
                init_flag_ += 2;
                ROS_INFO("Got UGV Acc!");
            }

            ugv_acc_ = ugv_acc;
            flag_rec_ugv_acc_ = true;
        }
    };

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "sim_acc_controller");

    sim_odom_controller::SimAccController controller;

    controller.loop();

}
