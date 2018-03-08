//
// Created by chengdaqian on 17-12-4.
//

#include <frontier_exploration/astar_costmap.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <boost/foreach.hpp>


namespace frontier_exploration{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

    Astar::Astar(costmap_2d::Costmap2D &costmap) : costmap_(costmap) {
        map_ = costmap_.getCharMap();
        size_x_ = costmap_.getSizeInCellsX();
        size_y_ = costmap_.getSizeInCellsY();
    }

    int Astar::calcG(AstarPoint *temp_start, AstarPoint *point) {
        int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? 10 : 14;
        int parentG = point->parent == NULL ? 0 : point->parent->G;
        return (parentG + extraG);
    }

    int Astar::calcH(AstarPoint *point, AstarPoint *end) {
        return (abs(end->x - point->x) + abs(end->y - point->y));
    }

    int Astar::calcF(AstarPoint *point) {
        return (point->G + point->H);
    }

    AstarPoint *Astar::getLeastFpoint() {
        if (!open_list_.empty())
        {
            AstarPoint* resPoint = open_list_.front();
            BOOST_FOREACH(AstarPoint *point, open_list_)
                if (point->F < resPoint->F)
                    resPoint = point;
            return resPoint;
        }
        return NULL;
    }

    AstarPoint *Astar::findPath(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner) {
        open_list_.push_back(new AstarPoint(startPoint.x, startPoint.y));
        while(!open_list_.empty())
        {
            AstarPoint* curPoint = getLeastFpoint();
            open_list_.remove(curPoint);
            close_list_.push_back(curPoint);

            std::vector<AstarPoint *> surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
            BOOST_FOREACH(AstarPoint *target, surroundPoints)
            {
                if (!isInList(open_list_, target))
                {
                    target->parent = curPoint;
                    target->G = calcG(curPoint, target);
                    target->H = calcH(target, &endPoint);
                    target->F = calcF(target);

                    open_list_.push_back(target);

                }else{
                    int tempG = calcG(curPoint, target);
                    if (tempG < target->G)
                    {
                        target->parent = curPoint;
                        target->G = tempG;
                        target->F = calcF(target);
                    }
                }
                AstarPoint *resPoint = isInList(open_list_, &endPoint);
                if (resPoint){
                    ROS_INFO("Found path!");
                    return resPoint;
                }

            }
        }

        return NULL;
    }

    std::list<AstarPoint *> Astar::getPath(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner) {
        AstarPoint *result = findPath(startPoint, endPoint, isIgnoreCorner);
        std::list<AstarPoint *> path;
        while(result)
        {
            path.push_front(result);
            result = result->parent;
        }

        // delete all points and clear lists
        AstarPoint* tmp;
        while (!open_list_.empty()){
            tmp = open_list_.front();
            delete tmp;
            open_list_.pop_front();
        }
        while (!close_list_.empty()){
            tmp = close_list_.front();
            delete tmp;
            close_list_.pop_front();
        }

        return path;
    }

    int Astar::getCost(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner) {
        AstarPoint *result = findPath(startPoint, endPoint, isIgnoreCorner);
        int result_G;
        if (result)
            result_G = result->G;
        else
            result_G = std::numeric_limits<int>::infinity();

        AstarPoint* tmp;
        while (!open_list_.empty()){
            tmp = open_list_.front();
            delete tmp;
            open_list_.pop_front();
        }
        while (!close_list_.empty()){
            tmp = close_list_.front();
            delete tmp;
            close_list_.pop_front();
        }

        return result_G;
    }

    AstarPoint *Astar::isInList(const std::list<AstarPoint *> &list, const AstarPoint *point) const{
        BOOST_FOREACH(AstarPoint* p, list)
            if(p->x == point->x && p->y == point->y)
                return p;
        return NULL;
    }

    bool Astar::isCanReach(const AstarPoint *point, const AstarPoint *target, bool isIgnoreCorner) const {
        if (target->x <0 || target->x >= size_x_ || target->y < 0 || target->y >= size_y_
            || target->x == point->x && target->y == point->y || isInList(close_list_, target))
            return false;

        unsigned int idx = costmap_.getIndex(target->x, target->y);
        if (map_[idx] == LETHAL_OBSTACLE)
            return false;

        if (abs(point->x-target->x)+abs(point->y-target->y)==1)
            return true;
        else{
            unsigned int idx1 = costmap_.getIndex(target->x, point->y), idx2 = costmap_.getIndex(point->x, target->y);
            if (map_[idx1] != LETHAL_OBSTACLE && map_[idx2] != LETHAL_OBSTACLE)
                return true;
            else
                return isIgnoreCorner;

        }
    }

    std::vector<AstarPoint *> Astar::getSurroundPoints(const AstarPoint *point, bool isIgnoreCorner) const {
        std::vector<AstarPoint *> surroundPoints;

        for(int x = point->x > 0 ? point->x - 1 : 0; x <= point->x + 1; x++)
            for(int y = point->y >0 ? point->y - 1 : 0; y <= point->y + 1; y++)
            {
                AstarPoint *new_point_ptr = new AstarPoint(x,y);
                if (isCanReach(point, new_point_ptr, isIgnoreCorner))
                    surroundPoints.push_back(new_point_ptr);
            }

        return surroundPoints;
    }

}