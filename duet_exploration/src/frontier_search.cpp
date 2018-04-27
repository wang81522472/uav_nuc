#include <duet_exploration/frontier_search.h>

#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>

#include <duet_exploration/costmap_tools.h>

namespace duet_exploration{

using simple_map_2d::LETHAL_OBSTACLE;
using simple_map_2d::NO_INFORMATION;
using simple_map_2d::FREE_SPACE;
using simple_map_2d::INSCRIBED_INFLATED_OBSTACLE;

FrontierSearch::FrontierSearch(simple_map_2d::SimpleMap2D &simple_map, int min_frontier_size, std::string &travel_point,
                               double wx_min, double wx_max, double wy_min, double wy_max) :
        simple_map_(simple_map), min_frontier_size_(min_frontier_size), travel_point_(travel_point),
        wx_min(wx_min), wx_max(wx_max), wy_min(wy_min), wy_max(wy_max)
{

}

std::list<Frontier> FrontierSearch::searchFrontierA(std::vector<unsigned char> &charmap_no_inflation)
{
    std::list<Frontier> frontier_list;

    //Sanity check that robot is inside costmap bounds before searching
    unsigned int mx,my;
    double wx, wy;

    //make sure map is consistent and locked for duration of search
    boost::unique_lock < simple_map_2d::SimpleMap2D::mutex_t > lock(*(simple_map_.getMutex()));

    size_x_ = simple_map_.getSizeInCellsX();
    size_y_ = simple_map_.getSizeInCellsY();

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
            simple_map_.indexToCells(idx, mx, my);
            simple_map_.mapToWorld(mx,my,wx,wy);
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


std::list<FrontierCell> FrontierSearch::searchFrontierBCell(std::vector<unsigned char> &charmap_no_inflation)
{
    std::list<FrontierCell> frontierB_cell_list;
    FrontierCell temp;
    unsigned int mx,my;
    double wx, wy;

    for (unsigned int i = 0; i < charmap_no_inflation.size(); i++){
        if (charmap_no_inflation[i] == NO_INFORMATION){

            simple_map_.indexToCells(i, mx, my);
            simple_map_.mapToWorld(mx,my,wx,wy);
            if (wx < wx_min + 1.0 || wy < wy_min + 1.0 || wx > wx_max - 1.0 || wy > wy_max - 1.0){
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
        simple_map_.indexToCells(initial_cell,ix,iy);
        geometry_msgs::Point temp;
        temp.x = ix; temp.y = iy;
        output.frontier_cells.push_back(temp);
        simple_map_.mapToWorld(ix,iy,output.travel_point.x,output.travel_point.y); // setup initial travel_point

        //push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);

        unsigned int mx, my;
        double wx, wy;

        while(!bfs.empty()){
            unsigned int idx = bfs.front();
            bfs.pop();

            //try adding cells in 8-connected neighborhood to frontier
            BOOST_FOREACH(unsigned int nbr, nhood8(idx, size_x_, size_y_)){
                            //check if neighbour is a potential frontier cell

                            if (charmap_no_inflation[nbr] == NO_INFORMATION){
                                if(frontier_flag[nbr]){
                                    continue;
                                }
                                simple_map_.indexToCells(nbr, mx, my);
                                simple_map_.mapToWorld(mx,my,wx,wy);
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
    simple_map_.indexToCells(initial_cell,ix,iy);
    geometry_msgs::Point temp;
    temp.x = ix; temp.y = iy;
    output.frontier_cells.push_back(temp);
    simple_map_.mapToWorld(ix,iy,output.travel_point.x,output.travel_point.y); // setup initial travel_point

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    //cache reference position in world coords
    unsigned int rx,ry;
    double reference_x, reference_y;
    simple_map_.indexToCells(reference,rx,ry);
    simple_map_.mapToWorld(rx,ry,reference_x,reference_y);

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, size_x_, size_y_)){
            //check if neighbour is a potential frontier cell
            if(isNewFrontierCell(nbr,frontier_flag)){

                //mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx,my;
                double wx,wy;
                simple_map_.indexToCells(nbr,mx,my);
                simple_map_.mapToWorld(mx,my,wx,wy);

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
    simple_map_.indexToCells(initial_cell,ix,iy);
    geometry_msgs::Point temp;
    temp.x = ix; temp.y = iy;
    output.frontier_cells.push_back(temp);
    simple_map_.mapToWorld(ix,iy,output.travel_point.x,output.travel_point.y); // setup initial travel_point

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    //cache reference position in world coords
    unsigned int rx,ry;
    double reference_x, reference_y;
    simple_map_.indexToCells(reference,rx,ry);
    simple_map_.mapToWorld(rx,ry,reference_x,reference_y);

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, size_x_, size_y_)){
                        //check if neighbour is a potential frontier cell
                        if(isNewFrontierCell(nbr, frontier_flag, charmap_no_inflation)){

                            //mark cell as frontier
                            frontier_flag[nbr] = true;
                            unsigned int mx,my;
                            double wx,wy;
                            simple_map_.indexToCells(nbr,mx,my);
                            simple_map_.mapToWorld(mx,my,wx,wy);

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

    simple_map_.indexToCells(idx, mx, my);
    simple_map_.mapToWorld(mx, my, wx, wy);
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

    simple_map_.indexToCells(idx, mx, my);
    simple_map_.mapToWorld(mx, my, wx, wy);
    if (wx < wx_min + 0.3 || wy < wy_min + 0.3 || wx > wx_max - 0.3 || wy > wy_max - 0.3){
        return false;
    }


    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx, size_x_, size_y_)){
        if (charmap_no_inflation[idx] == NO_INFORMATION && charmap_no_inflation[nbr] == LETHAL_OBSTACLE ){ //&& !charmap_no_inflation[idx]
            return true;
        }

    }

    return false;

}

}
