//
// Created by chengdaqian on 17-11-19.
//

#include <uav_costmap_obstacle_layer/uav_obstacle_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(uav_obstacle_layer::UAVObstacleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;

namespace uav_obstacle_layer
{

using namespace costmap_2d;

void UAVObstacleLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_), g_nh;
    rolling_window_ = layered_costmap_->isRolling();

    bool track_unknown_space;
    nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
    if (track_unknown_space)
        default_value_ = NO_INFORMATION;
    else
        default_value_ = FREE_SPACE;

    ObstacleLayer::matchSize();
    current_ = true;

    global_frame_ = layered_costmap_->getGlobalFrameID();
    double transform_tolerance;
    nh.param("transform_tolerance", transform_tolerance, 0.2);

    std::string topics_string;
    // get the topics that we'll subscribe to from the parameter server
    nh.param("observation_sources", topics_string, std::string(""));
    ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

    // get our tf prefix
    ros::NodeHandle prefix_nh;
    const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source)
    {
        ros::NodeHandle source_node(nh, source);

        // get the parameters for the specific topic
        double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
        std::string topic, sensor_frame, data_type;
        bool inf_is_valid, clearing, marking;

        source_node.param("topic", topic, source);
        source_node.param("sensor_frame", sensor_frame, std::string(""));
        source_node.param("observation_persistence", observation_keep_time, 0.0);
        source_node.param("expected_update_rate", expected_update_rate, 0.0);
        source_node.param("data_type", data_type, std::string("PointCloud"));

        source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
        source_node.param("inf_is_valid", inf_is_valid, false);
        source_node.param("clearing", clearing, false);
        source_node.param("marking", marking, true);

	ROS_ERROR_STREAM("frame: " << sensor_frame);

        // these two lines are modified
        source_node.param("min_obstacle_height", min_obstacle_height, -0.5);
        source_node.param("obstacle_height_threshold", obstacle_height_threshold_, 0.3);

        if (!sensor_frame.empty())
        {
            sensor_frame = tf::resolve(tf_prefix, sensor_frame);
        }

        if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
        {
            ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
            throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
        }

        std::string raytrace_range_param_name, obstacle_range_param_name;

        // get the obstacle range for the sensor
        double obstacle_range = 2.5;
        if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
        {
            source_node.getParam(obstacle_range_param_name, obstacle_range);
        }

        // get the raytrace range for the sensor
        double raytrace_range = 3.0;
        if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
        {
            source_node.getParam(raytrace_range_param_name, raytrace_range);
        }

        ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
                  sensor_frame.c_str());

        // create an observation buffer
        observation_buffers_.push_back(
                boost::shared_ptr < ObservationBuffer
                > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                         max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                         sensor_frame, transform_tolerance)));

        // check if we'll add this buffer to our marking observation buffers
        if (marking)
            marking_buffers_.push_back(observation_buffers_.back());

        // check if we'll also add this buffer to our clearing observation buffers
        if (clearing)
            clearing_buffers_.push_back(observation_buffers_.back());

        ROS_DEBUG(
                "Created an observation buffer for source %s, topic %s, global frame: %s, "
                        "expected update rate: %.2f, observation persistence: %.2f",
                source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

        // create a callback for the topic
        if (data_type == "LaserScan")
        {
            boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
            > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

            boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
            > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));

            if (inf_is_valid)
            {
                filter->registerCallback(
                        boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
            }
            else
            {
                filter->registerCallback(
                        boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));
            }

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);

            observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
        }
        else if (data_type == "PointCloud")
        {
            boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
            > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

            if (inf_is_valid)
            {
                ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
            }

            boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud>
            > filter(new tf::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50));
            filter->registerCallback(
                    boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
        }
        else
        {
            //用msg_filter建一个subscriber sub
            boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
            > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

            if (inf_is_valid)
            {
                ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
            }

            //建一个tf::MessageFilter，绑定callback
            boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
            > filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));
            filter->registerCallback(
                    boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
        }

        if (sensor_frame != "")
        {
            std::vector < std::string > target_frames;
            target_frames.push_back(global_frame_);
            target_frames.push_back(sensor_frame);
            observation_notifiers_.back()->setTargetFrames(target_frames);
        }
    }

    dsrv_ = NULL;
    setupDynamicReconfigure(nh);

    inflated_visited_ = std::vector<bool>(size_x_ * size_y_, 0);
}

void UAVObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                    double *min_x, double *min_y, double *max_x, double *max_y)
{

    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    if (!enabled_)
        return;
    useExtraBounds(min_x, min_y, max_x, max_y);

    bool current = true;
    std::vector<Observation> observations, clearing_observations;

    // get the marking observations
    current = current && getMarkingObservations(observations);

    // get the clearing observations
    current = current && getClearingObservations(clearing_observations);


    // update the global current status
    current_ = current;

    //raytrace freespace
    for (unsigned int i = 0; i < clearing_observations.size(); ++i)
    {
        raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
    }

    // clear points on the ground as freespace
    unsigned char* charMap = layered_costmap_->getCostmap()->getCharMap();
    for (std::vector<Observation>::const_iterator it = clearing_observations.begin(); it != clearing_observations.end(); ++it)
    {
        const Observation& obs = *it;

        const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

        for(unsigned int i = 0; i < cloud.points.size(); i++)
        {
            double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

            if (pz > obstacle_height_threshold_)
            {
                ROS_DEBUG("The point is an obstacle");
                continue;
            }

            unsigned int mx, my;

            if (!worldToMap(px,py,mx,my))
            {
                ROS_DEBUG("Computing map coords failed");
                continue;
            }

            unsigned int index = getIndex(mx, my);

            if (charMap[index] == INSCRIBED_INFLATED_OBSTACLE){
                inflated_visited_[index] = true;
            }

            costmap_[index] = FREE_SPACE;

            touch(px, py, min_x, min_y, max_x, max_y);
        }
    }

    // place the new obstacles into a priority queue... each with a priority of zero to begin with
    for (std::vector<Observation>::const_iterator it = observations.begin(); it!= observations.end(); ++it)
    {
        const Observation& obs = *it;

        const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

        double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

        for (unsigned int i = 0; i < cloud.points.size(); i++)
        {
            double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

            if (pz > max_obstacle_height_ || pz < obstacle_height_threshold_)
            {
                ROS_DEBUG("The point is too high or to low");
                continue;
            }

            double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x)
                             + (py - obs.origin_.y) * (py - obs.origin_.y)
                             + (pz - obs.origin_.z) * (pz - obs.origin_.z);
            if (sq_dist >= sq_obstacle_range)
            {
                ROS_DEBUG("The point is too far away");
                continue;
            }

            unsigned int mx, my;
            if (!worldToMap(px,py,mx,my))
            {
                ROS_DEBUG("Computing map coords failed");
                continue;
            }

            unsigned int index = getIndex(mx, my);
            costmap_[index] = LETHAL_OBSTACLE;
            touch(px, py, min_x, min_y, max_x, max_y);

            unsigned char* charMap = layered_costmap_->getCostmap()->getCharMap();
            if (index % size_x_ > 0 && charMap[index-1] == INSCRIBED_INFLATED_OBSTACLE)
                inflated_visited_[index-1] = true;
            if (index % size_x_ < size_x_ - 1 && charMap[index+1] == INSCRIBED_INFLATED_OBSTACLE)
                inflated_visited_[index+1] = true;
            if (index >= size_x_ && charMap[index - size_x_] == INSCRIBED_INFLATED_OBSTACLE)
                inflated_visited_[index - size_x_] = true;
            if (index < size_x_*(size_y_-1) && charMap[index + size_x_] == INSCRIBED_INFLATED_OBSTACLE){
                inflated_visited_[index + size_x_] = true;
            }

        }
    }


    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

}
