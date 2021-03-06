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
//#include <Eigen/SVD>
using namespace std;
using namespace cv;
using namespace aruco;
using namespace Eigen;

float MarkerSize = 0.1;
int query_id = 23;
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;

ros::Publisher pub_ar_odom;
ros::Publisher centroid_pub;

tf::TransformBroadcaster broadcaster_;
tf::Transform tf_uav_map_;


void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    ROS_INFO("Received image!");
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    Mat frame = bridge_ptr->image;

    int img_center_x = frame.size().width / 2;
    int img_center_y = frame.size().height / 2;
    //cout << "img_center_x: " << img_center_x << endl;
    //cout << "img_center_y: " << img_center_y << endl;

    MDetector.detect(frame, Markers);
    
    for (unsigned int i = 0; i < Markers.size(); i++)
    {

        ROS_WARN("Got one marker!");
        //cout << Markers[i] << endl;
        Markers[i].draw(frame, Scalar(0,0,255), 2);
        aruco::CvDrawingUtils::draw3dAxis(frame, Markers[i], CamParam);
     
        Markers[i].calculateExtrinsics(MarkerSize, CamParam);
        

        Point2f centroid = Markers[i].getCenter();

        geometry_msgs::Vector3 marker_center;
        marker_center.x = centroid.x;
        marker_center.y = centroid.y;
        marker_center.z = Markers[i].id;

        //centroid_pub.publish(marker_center);

        Mat rvec = Markers[i].Rvec;
        Mat tvec = Markers[i].Tvec;
        Mat R;
        Rodrigues(rvec, R);

        Matrix3d R_eigen;
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++) {
                R_eigen(j,k) = R.at<float>(j,k);
            }

        Quaterniond Q;
        Q = R_eigen;

        nav_msgs::Odometry odom_marker;

        odom_marker.header.stamp = img_msg->header.stamp;
        odom_marker.header.frame_id = "camera"; //TODO: ???
        odom_marker.pose.pose.position.x = tvec.at<float>(0,0);
        odom_marker.pose.pose.position.y = tvec.at<float>(1,0);
        odom_marker.pose.pose.position.z = tvec.at<float>(2,0);

        //cout << "tvec:  " << tvec.at<float>(0,0) << endl << tvec.at<float>(1,0) << endl << tvec.at<float>(2,0) << endl << "====" << endl;

        odom_marker.pose.pose.orientation.w = Q.w();
        odom_marker.pose.pose.orientation.x = Q.x();
        odom_marker.pose.pose.orientation.y = Q.y();
        odom_marker.pose.pose.orientation.z = Q.z();

        //Add id to the message
        odom_marker.twist.twist.linear.x = Markers[i].id;
        pub_ar_odom.publish(odom_marker);
    }
    //imshow("usb_image", frame);
    waitKey(5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_detector");
    ros::NodeHandle nh("~");
    string cam_cal;
    nh.getParam("cam_cal_file", cam_cal);
    CamParam.readFromXMLFile(cam_cal);
    
    centroid_pub = nh.advertise<geometry_msgs::Vector3>("/marker_center", 10);
    pub_ar_odom = nh.advertise<nav_msgs::Odometry>("/detected_markers", 10);
    ros::Subscriber sub_img = nh.subscribe("/zed/left/image_rect_color", 1, img_callback);
    //ros::Subscriber sub_query_id = nh.subscribe("/query_id", 1, query_id_callback);
    
    //cv::namedWindow("Marker Detector", 1);
    ros::spin();
}
