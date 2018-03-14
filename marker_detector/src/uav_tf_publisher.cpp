#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//#include <Eigen/SVD>
using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;

namespace marker_detector{

    class TfUAVMapPublisher{

    public:
        TfUAVMapPublisher() : nh_("~")
        {
            image_sub_ = nh_.subscribe("image", 1, &TfUAVMapPublisher::image_cb, this); // /zed/left/image_rect_color
            uav_odom_sub_ = nh_.subscribe("uav_odom", 1, &TfUAVMapPublisher::uav_odom_cb, this);

            nh_.param<double>("marker_size", marker_size_, 0.07);
            nh_.param<string>("uav_frame", uav_frame_, "world");
            nh_.param<string>("map_frame", map_frame_, "robot_1/map");
	    nh_.param<double>("init_yaw", init_yaw_, 0.0);
            std::string cam_cal;
            nh_.getParam("cam_cal_file", cam_cal);
            camera_param_.readFromXMLFile(cam_cal);

        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber image_sub_, uav_odom_sub_;
        tf::TransformBroadcaster broadcaster_;
        tf::Transform tf_uav_map_;

        double marker_size_;

        aruco::CameraParameters camera_param_;
        MarkerDetector marker_detector_;
        vector<Marker> markers_;

        geometry_msgs::Vector3 tum_;

        Eigen::Quaterniond Qum_;

        nav_msgs::Odometry uav_odom_;

        std::string uav_frame_, map_frame_;

	double init_yaw_;

        void image_cb(const sensor_msgs::ImageConstPtr &img_rec){

            ROS_INFO("Received image!");
            cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_rec, sensor_msgs::image_encodings::MONO8);
            Mat frame = bridge_ptr->image;

            marker_detector_.detect(frame, markers_);

            for (unsigned int i = 0; i < markers_.size(); i++)
            {
                markers_[i].calculateExtrinsics(marker_size_, camera_param_);

                /*
                ROS_WARN("Got one marker!");
                //cout << Markers[i] << endl;
                Markers[i].draw(frame, Scalar(0,0,255), 2);
                aruco::CvDrawingUtils::draw3dAxis(frame, Markers[i], CamParam);

                Point2f centroid = Markers[i].getCenter();

                geometry_msgs::Vector3 marker_center;
                marker_center.x = centroid.x;
                marker_center.y = centroid.y;
                marker_center.z = Markers[i].id; // translation

                //centroid_pub.publish(marker_center);
                 */

		Eigen::Matrix4d Tzb;
                Tzb << 1, 0, 0, -0.1, 0, -1, 0, 0, 0, 0, -1, -0.07, 0, 0, 0, 1;

                Mat rvec = markers_[i].Rvec;
                Mat tvec = markers_[i].Tvec;
                Mat R;
                Rodrigues(rvec, R);

                Matrix3d R_eigen, UAV_R;
                for(int j=0;j<3;j++)
                    for(int k=0;k<3;k++) {
                        R_eigen(j,k) = R.at<float>(j,k);
                    }

		Vector3d rpy = R_eigen.eulerAngles(0,1,2);
		cout << "rpy: \n" << rpy << endl; 
                Quaterniond Q, Qum;
                Q = R_eigen; // rotation

                Eigen::Matrix4d Tbm, Puav, Tum;


		Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd yawAngle(init_yaw_, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd pitchAngle(0.0, Eigen::Vector3d::UnitX());
		Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
                Tbm.block(0,0,3,3) = q.matrix();
                Tbm.col(3) << -tvec.at<float>(0,0), -tvec.at<float>(1,0), -tvec.at<float>(2,0), 1;
                Quaterniond uav_q (uav_odom_.pose.pose.orientation.w, uav_odom_.pose.pose.orientation.x,
                                   uav_odom_.pose.pose.orientation.y, uav_odom_.pose.pose.orientation.z);
                UAV_R = uav_q.toRotationMatrix();
                Puav.block(0,0,3,3) = UAV_R;
                Puav.col(3) << uav_odom_.pose.pose.position.x, uav_odom_.pose.pose.position.y, uav_odom_.pose.pose.position.z, 1;


                Tum = Puav * Tbm;
		cout<< "Tum:" << endl << Puav << endl<< "Tzb" << endl << Tzb << endl << "Tbm" << endl << Tbm << endl << "Tum" << endl << Tum << endl;
                Matrix3d tmp_R = Tum.block(0,0,3,3);
                Qum = tmp_R;
                geometry_msgs::Vector3 tum, tmp;
                tum.x = Tum(0,3);
                tum.y = Tum(1,3);
                tum.z = Tum(2,3);

                tf::Quaternion tfq(Qum.x(), Qum.y(), Qum.z(), Qum.w());
                tf_uav_map_.setRotation(tfq);
                tf_uav_map_.setOrigin(tf::Vector3(tum.x, tum.y, tum.z));


		ROS_INFO_STREAM("z: \n" << tvec);

                while(true){
                    ROS_WARN_STREAM("Got a shot! Yaw is " << init_yaw_); 
                    broadcaster_.sendTransform(tf::StampedTransform(tf_uav_map_, ros::Time::now(), map_frame_, uav_frame_));
                    ros::Duration(1).sleep();
                    //ros::spinOnce();
                }
            }
        }

        void uav_odom_cb(const nav_msgs::Odometry &odom_rec){
            uav_odom_ = odom_rec;
        }

    };

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_detector");

    marker_detector::TfUAVMapPublisher publisher;

    ros::spin();
}
