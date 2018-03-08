#include "ros/ros.h"
#include <cstdlib>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "make_plan_test");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("/robot_1/move_base_node/make_plan");
    nav_msgs::GetPlan srv;

    geometry_msgs::PoseStamped start, goal;
    start.header.frame_id = goal.header.frame_id = "robot_1/map";
    start.pose.position.x = start.pose.position.y = 1.0;
    goal.pose.position.x = goal.pose.position.y = 7.0;
    srv.request.start = start;
    srv.request.goal  = goal;

    while (ros::ok()) {
        if (client.call(srv)) {
            ROS_INFO_STREAM("Received plan's length: " << srv.response.plan.poses.size());
        } else {
            ROS_ERROR("Failed to call");
            return 1;
        }
    }

    return 0;
}