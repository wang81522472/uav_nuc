#include <simple_map_2d/simple_map_2d.h>
#include <cstdio>
#include <ros/ros.h>

namespace simple_map_2d
{
    SimpleMap2D::SimpleMap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                             double origin_x, double origin_y, unsigned char default_value) :
            size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x),
            origin_y_(origin_y), costmap_(NULL), default_value_(default_value)
    {
        access_ = new mutex_t();
        initMaps(size_x_, size_y_);
        //inflation_layer_ = new MapInflation(this);
    }

    void SimpleMap2D::initMaps(unsigned int size_x, unsigned int size_y) {
        boost::unique_lock<mutex_t> lock(*access_);

        costmap_ = new unsigned char[size_x * size_y];
        inflated_costmap_ = new unsigned char[size_x * size_y];

        for (unsigned int idx = 0; idx < size_x * size_y; idx++){
            costmap_[idx] = inflated_costmap_[idx] = default_value_;
        }
    }

    SimpleMap2D::SimpleMap2D() :
            size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
    {
        access_ = new mutex_t();
    }

    SimpleMap2D::~SimpleMap2D()
    {
        deleteMap();
        delete access_;
        //delete inflation_layer_;
    }

    void SimpleMap2D::deleteMap(){
        boost::unique_lock<mutex_t> lock(*access_);
        delete[] costmap_;
        delete[] inflated_costmap_;
        costmap_ = inflated_costmap_ = NULL;
    }

    unsigned char SimpleMap2D::getCost(unsigned int idx) const {
        return costmap_[idx];
    }
    unsigned char SimpleMap2D::getCost(unsigned int mx, unsigned int my) const {
        return costmap_[getIndex(mx, my)];
    }
    unsigned char SimpleMap2D::getCostInflated(unsigned int idx) const {
        return inflated_costmap_[idx];
    }
    unsigned char SimpleMap2D::getCostInflated(unsigned int mx, unsigned int my) const {
        return inflated_costmap_[getIndex(mx, my)];
    }

    void SimpleMap2D::setCost(unsigned int idx, unsigned char cost){
        costmap_[idx] = cost;
    }
    void SimpleMap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
    {
        costmap_[getIndex(mx, my)] = cost;
    }
    void SimpleMap2D::setCostInflated(unsigned int idx, unsigned char cost) {
        inflated_costmap_[idx] = cost;
    }
    void SimpleMap2D::setCostInflated(unsigned int mx, unsigned int my, unsigned char cost) {
        inflated_costmap_[getIndex(mx, my)] = cost;
    }

    void SimpleMap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy){
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
    }

    bool SimpleMap2D::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) {
        if (wx < origin_x_ || wy < origin_y_)
            return false;

        mx = (int)((wx - origin_x_) / resolution_);
        my = (int)((wy - origin_y_) / resolution_);

        if (mx < size_x_ && my < size_y_)
            return true;

        return false;
    }

    unsigned int SimpleMap2D::cellDistance(double world_dist)
    {
        double cells_dist = ceil(world_dist / resolution_);
        if (cells_dist < 0.0)
            cells_dist = 0.0;
        return (unsigned int)cells_dist;
    }



    bool SimpleMap2D::setPolygonEdgeCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value)
    {
        // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
        MapLocation loc, loc_parent;
        if (!worldToMap(polygon[0].x, polygon[0].y, loc_parent.x, loc_parent.y))
            return false;
        for (unsigned int i = 1; i < polygon.size(); ++i)
        {
            if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y))
                return false;
            raytraceLine(cost_value, loc_parent.x, loc_parent.y, loc.x, loc.y);
            loc_parent.x = loc.x;
            loc_parent.y = loc.y;
        }

        return true;
    }

    /*
    void SimpleMap2D::copyAndInflate(){
        // copy data where there is conflic about LethalObstacle
        for (unsigned int idx = 0; idx < size_y_ * size_x_; idx++){
            if (costmap_[idx] == LETHAL_OBSTACLE && inflated_costmap_[idx] != LETHAL_OBSTACLE
                || costmap_[idx] != LETHAL_OBSTACLE && inflated_costmap_[idx] == LETHAL_OBSTACLE)
                inflated_costmap_[idx] = costmap_[idx];
        }

        // get inflated map
        inflation_layer_->updateCosts(0,0,size_x_, size_y_);
    }
     */


}