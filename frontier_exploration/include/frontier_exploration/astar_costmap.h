//
// Created by chengdaqian on 17-12-4.
//

#ifndef FRONTIER_EXPLORATION_ASTAR_COSTMAP_H
#define FRONTIER_EXPLORATION_ASTAR_COSTMAP_H

#include <list>
#include <vector>
#include <costmap_2d/costmap_2d.h>
#include <iostream>
#include <ros/ros.h>

namespace frontier_exploration {

struct AstarPoint {
    unsigned int x, y;
    int F, G, H;
    AstarPoint *parent;

    AstarPoint(int _x = 0, int _y = 0) : x(_x), y(_y), F(0), G(0), H(0), parent(NULL) {};
};

class Astar {

public:
    Astar(costmap_2d::Costmap2D &costmap);

    std::list<AstarPoint *> getPath(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner);

    int getCost(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner);

private:
    costmap_2d::Costmap2D &costmap_;
    unsigned char *map_;
    unsigned int size_x_, size_y_;

    std::list<AstarPoint *> open_list_;
    std::list<AstarPoint *> close_list_;

    AstarPoint *findPath(AstarPoint &startPoint, AstarPoint &endPoint, bool isIgnoreCorner);

    std::vector<AstarPoint *> getSurroundPoints(const AstarPoint *point, bool isIgnoreCorner) const;

    bool isCanReach(const AstarPoint *point, const AstarPoint *target, bool isIgnoreCorner) const;

    AstarPoint *isInList(const std::list<AstarPoint *> &list, const AstarPoint *point) const;

    AstarPoint *getLeastFpoint();

    int calcG(AstarPoint *temp_start, AstarPoint *point);

    int calcH(AstarPoint *point, AstarPoint *end);

    int calcF(AstarPoint *point);
};

}

#endif //FRONTIER_EXPLORATION_ASTAR_COSTMAP_H
