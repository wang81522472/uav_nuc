//
// Created by chengdaqian on 18-4-15.
//

#include <ros/ros.h>
#include <simple_map_2d/simple_map_2d_ros.h>
#include <simple_map_2d/map_inflation.h>

namespace simple_map_2d
{
    class DuetTest{
    public:
        DuetTest()
        {
            simple_map_ros_ = boost::shared_ptr<SimpleMap2DROS>(new SimpleMap2DROS("simple_map_node", "/robot_1/odom"));
            map_inflation_ = boost::shared_ptr<MapInflation>(new MapInflation(simple_map_ros_->map_));
        }

        void test_loop(){
            while(ros::ok()){
                ROS_INFO("Test flag L1");
                ros::spinOnce();
                ROS_INFO("Test flag L2");
                map_inflation_->updateCosts(0,0,300,300);
                simple_map_ros_->pubMap();
                ROS_INFO("Test flag L3");
                ros::Duration(0.25).sleep();
                ROS_INFO("Test flag L4");
            }
        }


    private:
        boost::shared_ptr<SimpleMap2DROS> simple_map_ros_;
        boost::shared_ptr<MapInflation>   map_inflation_;
    };
}

int main(int argc, char** argv){

    ros::init(argc, argv, "duet_test_node");

    simple_map_2d::DuetTest test;

    ROS_INFO("Test flag 1");

    test.test_loop();
}