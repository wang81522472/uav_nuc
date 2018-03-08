#include <frontier_exploration/frontier_search.h>

#include <costmap_2d/costmap_2d.h>
#include<costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/costmap_tools.h>

namespace frontier_exploration{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D &costmap, int min_frontier_size, std::string &travel_point,
                               double wx_min, double wx_max, double wy_min, double wy_max) :
        costmap_(costmap), min_frontier_size_(min_frontier_size), travel_point_(travel_point),
        wx_min(wx_min), wx_max(wx_max), wy_min(wy_min), wy_max(wy_max)
{

    //nh_.param<double>("/explore_server/explore_costmap/inflation/inflation_radius", inflation_radius_, 1.0);
    //inflation_radius_ *= 1.5;
    //nh_.param<double>("/explore_server/explore_costmap/resolution", resolution_, 0.1);
}

std::list<Frontier> FrontierSearch::searchFrontierA(std::vector<unsigned char> &charmap_no_inflation)
{
    std::list<Frontier> frontier_list;

    //Sanity check that robot is inside costmap bounds before searching
    unsigned int mx,my;
    double wx, wy;

    //make sure map is consistent and locked for duration of search
    boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock(*(costmap_.getMutex()));

    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    //initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    //new method to replace bfs search
    Frontier temp;
    geometry_msgs::Point temp_point;
    for (unsigned int idx = 0;idx < size_x_ * size_y_;idx++){
        if (charmap_no_inflation[idx] == NO_INFORMATION){
            if(frontier_flag[idx]){
                continue;
            }
            costmap_.indexToCells(idx, mx, my);
            costmap_.mapToWorld(mx,my,wx,wy);
            if (wx < wx_min + 0.3 || wy < wy_min + 0.3 || wx > wx_max - 0.3 || wy > wy_max - 0.3){
                continue; // return false if cell is out of boundary
            }
            if(idx % size_x_ > 0 && charmap_no_inflation[idx-1] == FREE_SPACE
               || idx % size_x_ < size_x_ - 1 && charmap_no_inflation[idx + 1] == FREE_SPACE
               || idx >= size_x_ && charmap_no_inflation[idx - size_x_] == FREE_SPACE
               || idx < size_x_*(size_y_-1) && charmap_no_inflation[idx + size_x_] == FREE_SPACE){

                frontier_flag[idx] = true;

                Frontier new_frontier = buildNewFrontier(idx, frontier_flag, charmap_no_inflation);
                if (new_frontier.size > min_frontier_size_) {
                    frontier_list.push_back(new_frontier);
                    //ROS_INFO("FronA Usable");
                }
            }
        }
    }
    return frontier_list;
}

std::list<Frontier> FrontierSearch::searchFrontierAFrom(geometry_msgs::Point position){

    std::list<Frontier> frontier_list;

    //Sanity check that robot is inside costmap bounds before searching
    unsigned int mx,my;
    if (!costmap_.worldToMap(position.x,position.y,mx,my)){
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    //make sure map is consistent and locked for duration of search
    boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock(*(costmap_.getMutex()));

    map_ = costmap_.getCharMap();
    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    //initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    ROS_INFO("Time flag FronA before BFS");

    //initialize breadth first search
    std::queue<unsigned int> bfs;

    //find closest clear cell to start search
    unsigned int clear, pos = costmap_.getIndex(mx,my);
    if(nearestCell(clear, pos, FREE_SPACE, costmap_)){
        bfs.push(clear);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;



    while(!bfs.empty()) {
        unsigned int idx = bfs.front();
        bfs.pop();

        //iterate over 4-connected neighbourhood
        BOOST_FOREACH(unsigned nbr, nhood4(idx, costmap_)) {
                        //add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
                        if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
                            visited_flag[nbr] = true;
                            bfs.push(nbr);
                            //check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                        } else if (isNewFrontierCell(nbr, frontier_flag)) {// if nbr not free, check if new frontier
                            frontier_flag[nbr] = true;
                            //ROS_INFO("Time flag before Build New FronA");
                            Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                            //ROS_INFO("Time flag after Build New FronA");

                            if (new_frontier.size > min_frontier_size_) {
                                frontier_list.push_back(new_frontier);
                                //ROS_INFO("FronA Usable");
                            }
                        }
                    }
    }

    ROS_INFO("Time flag FronA after BFS");
    return frontier_list;

}

std::list<Frontier> FrontierSearch::searchFrontierBFrom(geometry_msgs::Point position, std::vector<unsigned char>& charmap_no_inflation){

    std::list<Frontier> frontier_list;

    //Sanity check that robot is inside costmap bounds before searching
    unsigned int mx,my;
    if (!costmap_.worldToMap(position.x,position.y,mx,my)){
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    //make sure map is consistent and locked for duration of search
    boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock(*(costmap_.getMutex()));

    map_ = costmap_.getCharMap();
    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    //initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    //initialize breadth first search
    std::queue<unsigned int> bfs;

    //find closest clear cell to start search

    unsigned int obstacle, pos = costmap_.getIndex(mx,my);

    if(nearestCell(obstacle, pos, LETHAL_OBSTACLE, costmap_)){
        bfs.push(obstacle);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby inflated cell to start search for FrontierB");
    }
    bfs.push(pos);

    visited_flag[bfs.front()] = true;



    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //iterate over 4-connected neighbourhood
        BOOST_FOREACH(unsigned nbr, nhood4(idx, costmap_)){
                        //add to queue all occupied, unvisited cells, use ascending search in case initialized on non-free cell
                        //if(map_[nbr] >= map_[idx] && !visited_flag[nbr]){
                        if(!visited_flag[nbr]){
                            visited_flag[nbr] = true;
                            bfs.push(nbr);
                            //check if cell is new frontierB cell (unvisited, NO_INFORMATION, occupied neighbour)
                        }else if(isNewFrontierCell(nbr, frontier_flag, charmap_no_inflation)){
                            frontier_flag[nbr] = true;
                            Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag, charmap_no_inflation);
                            if(new_frontier.size > min_frontier_size_){
                                frontier_list.push_back(new_frontier);
                            }
                        }
                    }
    }

    ROS_INFO("Time flag FronB after BFS");

    return frontier_list;

}

std::list<FrontierCell> FrontierSearch::searchFrontierBCell(std::vector<unsigned char> &charmap_no_inflation)
{
    std::list<FrontierCell> frontierB_cell_list;
    FrontierCell temp;
    unsigned int mx,my;
    double wx, wy;

    for (unsigned int i = 0; i < charmap_no_inflation.size(); i++){
        if (charmap_no_inflation[i] == NO_INFORMATION){

            costmap_.indexToCells(i, mx, my);
            costmap_.mapToWorld(mx,my,wx,wy);
            if (wx < wx_min + 0.3 || wy < wy_min + 0.3 || wx > wx_max - 0.3 || wy > wy_max - 0.3){
                continue;
            }

            if(i % size_x_ > 0 && charmap_no_inflation[i-1] == LETHAL_OBSTACLE
               || i % size_x_ < size_x_ - 1 && charmap_no_inflation[i + 1] == LETHAL_OBSTACLE
               || i >= size_x_ && charmap_no_inflation[i - size_x_] == LETHAL_OBSTACLE
               || i < size_x_*(size_y_-1) && charmap_no_inflation[i + size_x_] == LETHAL_OBSTACLE){

                temp.frontier_cell_point.x = mx;
                temp.frontier_cell_point.y = my;
                temp.utility = 1.0;
                frontierB_cell_list.push_back(temp);
            }
        }
    }
    return frontierB_cell_list;
}

    Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, std::vector<bool>& frontier_flag, std::vector<unsigned char> &charmap_no_inflation){

        //initialize frontier structure
        Frontier output;
        geometry_msgs::Point centroid, middle;
        output.size = 1;
        output.min_distance = std::numeric_limits<double>::infinity();

        //record initial contact point for frontier
        unsigned int ix, iy;
        costmap_.indexToCells(initial_cell,ix,iy);
        geometry_msgs::Point temp;
        temp.x = ix; temp.y = iy;
        output.frontier_cells.push_back(temp);
        costmap_.mapToWorld(ix,iy,output.travel_point.x,output.travel_point.y); // setup initial travel_point

        //push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);

        unsigned int mx, my;
        double wx, wy;

        while(!bfs.empty()){
            unsigned int idx = bfs.front();
            bfs.pop();

            //try adding cells in 8-connected neighborhood to frontier
            BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
                            //check if neighbour is a potential frontier cell

                            if (charmap_no_inflation[nbr] == NO_INFORMATION){
                                if(frontier_flag[nbr]){
                                    continue;
                                }
                                costmap_.indexToCells(nbr, mx, my);
                                costmap_.mapToWorld(mx,my,wx,wy);
                                if (wx < wx_min + 0.3 || wy < wy_min + 0.3 || wx > wx_max - 0.3 || wy > wy_max - 0.3){
                                    continue; // return false if cell is out of boundary
                                }
                                if(nbr % size_x_ > 0 && charmap_no_inflation[nbr-1] == FREE_SPACE
                                   || nbr % size_x_ < size_x_ - 1 && charmap_no_inflation[nbr + 1] == FREE_SPACE
                                   || nbr >= size_x_ && charmap_no_inflation[nbr - size_x_] == FREE_SPACE
                                   || nbr < size_x_*(size_y_-1) && charmap_no_inflation[nbr + size_x_] == FREE_SPACE){

                                    frontier_flag[nbr] = true;

                                    output.size++;

                                    //push cell into Frontier's frontier_cell list

                                    temp.x = mx; temp.y = my;
                                    output.frontier_cells.push_back(temp);

                                    //add to queue for breadth first search
                                    bfs.push(nbr);
                                }
                            }
                        }
        }

        return output;
    }

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag){

    //initialize frontier structure
    Frontier output;
    geometry_msgs::Point centroid, middle;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();



    //record initial contact point for frontier
    unsigned int ix, iy;
    costmap_.indexToCells(initial_cell,ix,iy);
    geometry_msgs::Point temp;
    temp.x = ix; temp.y = iy;
    output.frontier_cells.push_back(temp);
    costmap_.mapToWorld(ix,iy,output.travel_point.x,output.travel_point.y); // setup initial travel_point

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    //cache reference position in world coords
    unsigned int rx,ry;
    double reference_x, reference_y;
    costmap_.indexToCells(reference,rx,ry);
    costmap_.mapToWorld(rx,ry,reference_x,reference_y);

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
            //check if neighbour is a potential frontier cell
            if(isNewFrontierCell(nbr,frontier_flag)){

                //mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx,my;
                double wx,wy;
                costmap_.indexToCells(nbr,mx,my);
                costmap_.mapToWorld(mx,my,wx,wy);

                //update frontier size
                output.size++;

                //update centroid of frontier
                centroid.x += wx;
                centroid.y += wy;

                //push cell into Frontier's frontier_cell list
                geometry_msgs::Point temp;
                temp.x = mx; temp.y = my;
                output.frontier_cells.push_back(temp);

                //determine frontier's distance from robot, going by closest gridcell to robot
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance){
                    output.min_distance = distance;
                    middle.x = wx;
                    middle.y = wy;
                }

                //add to queue for breadth first search
                bfs.push(nbr);
            }
        }
    }

    //average out frontier centroid
    centroid.x /= output.size;
    centroid.y /= output.size;

    if(travel_point_ == "closest"){
        // point already set
    }else if(travel_point_ == "middle"){
        output.travel_point = middle;
    }else if(travel_point_ == "centroid"){
        output.travel_point = centroid;
    }else{
        ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
        // point already set
    }

    return output;
}




Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                          std::vector<bool>& frontier_flag, std::vector<unsigned char>& charmap_no_inflation){

    //initialize frontier structure
    Frontier output;
    geometry_msgs::Point centroid, middle;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    //record initial contact point for frontier
    unsigned int ix, iy;
    costmap_.indexToCells(initial_cell,ix,iy);
    geometry_msgs::Point temp;
    temp.x = ix; temp.y = iy;
    output.frontier_cells.push_back(temp);
    costmap_.mapToWorld(ix,iy,output.travel_point.x,output.travel_point.y); // setup initial travel_point

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    //cache reference position in world coords
    unsigned int rx,ry;
    double reference_x, reference_y;
    costmap_.indexToCells(reference,rx,ry);
    costmap_.mapToWorld(rx,ry,reference_x,reference_y);

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
                        //check if neighbour is a potential frontier cell
                        if(isNewFrontierCell(nbr, frontier_flag, charmap_no_inflation)){

                            //mark cell as frontier
                            frontier_flag[nbr] = true;
                            unsigned int mx,my;
                            double wx,wy;
                            costmap_.indexToCells(nbr,mx,my);
                            costmap_.mapToWorld(mx,my,wx,wy);

                            //update frontier size
                            output.size++;

                            //update centroid of frontier
                            centroid.x += wx;
                            centroid.y += wy;

                            //push cell into Frontier's frontier_cell list
                            geometry_msgs::Point temp;
                            temp.x = mx; temp.y = my;
                            output.frontier_cells.push_back(temp);

                            //determine frontier's distance from robot, going by closest gridcell to robot
                            double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                            if(distance < output.min_distance){
                                output.min_distance = distance;
                                middle.x = wx;
                                middle.y = wy;
                            }

                            //add to queue for breadth first search
                            bfs.push(nbr);
                        }
                    }
    }

    //average out frontier centroid
    centroid.x /= output.size;
    centroid.y /= output.size;

    if(travel_point_ == "closest"){
        // point already set
    }else if(travel_point_ == "middle"){
        output.travel_point = middle;
    }else if(travel_point_ == "centroid"){
        output.travel_point = centroid;
    }else{
        ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
        // point already set
    }

    return output;
}

    // is new frontiercellA
bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag) {

        if(frontier_flag[idx]){
            return false;
        }
    unsigned int mx, my;
    double wx, wy;

    costmap_.indexToCells(idx, mx, my);
    costmap_.mapToWorld(mx, my, wx, wy);
    if (wx < wx_min + 0.3 || wy < wy_min + 0.3 || wx > wx_max - 0.3 || wy > wy_max - 0.3){
        return false; // return false if cell is out of boundary
    }


    BOOST_FOREACH(unsigned int nbr, nhood4(idx, size_x_, size_y_)){
        if (map_[idx] == NO_INFORMATION && map_[nbr] == FREE_SPACE){ //type 0 refers to FrontierA
            return true;
        }
    }

    return false;

}

    // is new frontiercellB
bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag, std::vector<unsigned char>& charmap_no_inflation){

    //check that cell is unknown and not already marked as frontier
    if(frontier_flag[idx]){
        return false;
    }
    unsigned int mx, my;
    double wx, wy;

    costmap_.indexToCells(idx, mx, my);
    costmap_.mapToWorld(mx, my, wx, wy);
    if (wx < wx_min + 0.3 || wy < wy_min + 0.3 || wx > wx_max - 0.3 || wy > wy_max - 0.3){
        return false;
    }


    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx, costmap_)){
        if (charmap_no_inflation[idx] == NO_INFORMATION && charmap_no_inflation[nbr] == LETHAL_OBSTACLE ){ //&& !charmap_no_inflation[idx]
            return true;
        }

    }

    return false;

}

}
