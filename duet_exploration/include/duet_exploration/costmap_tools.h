#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>

namespace duet_exploration{

/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */

    std::vector<unsigned int> nhood4(unsigned int idx, unsigned int size_x_, unsigned int size_y_){
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


/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
    std::vector<unsigned int> nhood8(unsigned int idx, unsigned int size_x_, unsigned int size_y_){
        //get 8-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out = nhood4(idx, size_x_, size_y_);

        if (idx > size_x_ * size_y_ -1){
            return out;
        }

        if(idx % size_x_ > 0 && idx >= size_x_){
            out.push_back(idx - 1 - size_x_);
        }
        if(idx % size_x_ > 0 && idx < size_x_*(size_y_-1)){
            out.push_back(idx - 1 + size_x_);
        }
        if(idx % size_x_ < size_x_ - 1 && idx >= size_x_){
            out.push_back(idx + 1 - size_x_);
        }
        if(idx % size_x_ < size_x_ - 1 && idx < size_x_*(size_y_-1)){
            out.push_back(idx + 1 + size_x_);
        }

        return out;

    }


    int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }

}
#endif