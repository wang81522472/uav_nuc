//
// Created by chengdaqian on 18-2-20.
//

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>

#include <std_srvs/Empty.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/GetNextAirGroundFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/BlacklistPoint.h>


#include <geometry_msgs/PointStamped.h>

#include <frontier_exploration/geometry_tools.h>

#include <costmap_2d/footprint.h>
#include <tf/transform_listener.h>

#include <ros/wall_timer.h>

#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>

#include <string>

namespace frontier_exploration
{
    class ExploreStarter{

    public:
        ExploreStarter() : private_nh_("~"), tf_listener_(ros::Duration(10.0))
        {

            // code for server


            ROS_WARN("before init costmap");

            explore_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_));

            ROS_WARN("after init costmap");

            ros::ServiceClient updateBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>("explore_costmap/air_ground_explore/update_boundary_polygon");
            ros::ServiceClient getNextAirGroundFrontier = private_nh_.serviceClient<frontier_exploration::GetNextAirGroundFrontier>("explore_costmap/air_ground_explore/get_next_air_ground_frontier");

            // code for client
            private_nh_.param<std::string>("map_frame", map_frame_, "/robot_1/map");
            input_.header.frame_id = map_frame_;
            point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("exploration_polygon_marker", 10);
            //point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), boost::bind(&ExploreStarter::vizPubCb, this));
            ROS_INFO("Please use the 'Point' tool in Rviz to select an exporation boundary.");


            // initial goal: only for testing purpose
            bool initial_goal_flag;
            double wx_min, wy_min, wx_max, wy_max;

            nh_.param<double>("/wx_min", wx_min, -1.0);
            nh_.param<double>("/wx_max", wx_max, 5.0);
            nh_.param<double>("/wy_min", wy_min, -1.0);
            nh_.param<double>("/wy_max", wy_max, 5.0);

            ROS_INFO("Setting up initial goal_area.");
            geometry_msgs::Point32 point_bl, point_br, point_tl, point_tr;
            point_bl.x = point_tl.x = wx_min;
            point_bl.y = point_br.y = wy_min;
            point_tl.y = point_tr.y = wy_max;
            point_tr.x = point_br.x = wx_max;

            input_.polygon.points.push_back(point_bl);
            input_.polygon.points.push_back(point_br);
            input_.polygon.points.push_back(point_tr);
            input_.polygon.points.push_back(point_tl);
            input_.polygon.points.push_back(point_bl);

            //set boundary
            ROS_INFO("Before set bound");

            explore_costmap_ros_->resetLayers();

            if(!updateBoundaryPolygon.waitForExistence() || !getNextAirGroundFrontier.waitForExistence()){
                ROS_WARN("No srv hosts!!!");
                return;
            }

            ROS_INFO("Services exist!");


            frontier_exploration::UpdateBoundaryPolygon update_boundary_srv;
            update_boundary_srv.request.explore_boundary = input_;
            if(updateBoundaryPolygon.call(update_boundary_srv)){
                ROS_INFO("Region boundary set");
            }else{
                ROS_ERROR("Failed to set region boundary");
                return;
            }
            ROS_INFO("after set bound");

            frontier_exploration::GetNextAirGroundFrontier srv;
            srv.request.start_pose_air.pose.position.x = 1.0;
            srv.request.start_pose_ground.header.stamp = ros::Time::now();

            if(getNextAirGroundFrontier.call(srv)){
                ROS_WARN("Called");
            }else{
                ROS_WARN("Call failed");
            }

        }

    private:
        ros::NodeHandle nh_, private_nh_;

        ros::Publisher point_viz_pub_;
        ros::WallTimer point_viz_timer_;
        geometry_msgs::PolygonStamped input_;

        tf::TransformListener tf_listener_;

        boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;

        std::string map_frame_;

        bool waiting_for_center_;

        void vizPubCb(){

            visualization_msgs::Marker points, line_strip;

            points.header = line_strip.header = input_.header;
            points.ns = line_strip.ns = "explore_points";

            points.id = 0;
            line_strip.id = 1;

            points.type = visualization_msgs::Marker::SPHERE_LIST;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;

            if(!input_.polygon.points.empty()){

                points.action = line_strip.action = visualization_msgs::Marker::ADD;
                points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

                points.scale.x = points.scale.y = 0.1;
                line_strip.scale.x = 0.05;

                BOOST_FOREACH(geometry_msgs::Point32 point, input_.polygon.points){
                                line_strip.points.push_back(costmap_2d::toPoint(point));
                                points.points.push_back(costmap_2d::toPoint(point));
                            }

                if(waiting_for_center_){
                    line_strip.points.push_back(costmap_2d::toPoint(input_.polygon.points.front()));
                    points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
                }else{
                    points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;
                }
            }else{
                points.action = line_strip.action = visualization_msgs::Marker::DELETE;
            }
            point_viz_pub_.publish(points);
            point_viz_pub_.publish(line_strip);

        }

    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");

    frontier_exploration::ExploreStarter starter;

    ros::spin();
}