#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <duet_exploration/Frontier.h>
#include <duet_exploration/FrontierCell.h>
#include <simple_map_2d/simple_map_2d.h>
#include <ros/ros.h>


namespace duet_exploration{

/**
 * @brief Thread-safe implementation of a frontier-search task for an input costmap.
 */
class FrontierSearch{

public:

    /**
     * @brief Constructor for search task
     * @param costmap Reference to costmap data to search.
     * @param min_frontier_size The minimum size to accept a frontier
     * @param travel_point The requested travel point (closest|middle|centroid)
     */
    FrontierSearch(simple_map_2d::SimpleMap2D &simple_map, int min_frontier_size, std::string &travel_point,
                   double wx_min, double wx_max, double wy_min, double wy_max);

    /**
     * @brief Runs search implementation, outward from the start position
     * @param position Initial position to search from
     * @return List of frontiers, if any
     */
    std::list<Frontier> searchFrontierA(std::vector<unsigned char> &charmap_no_inflation);

    std::list<Frontier> searchFrontierBFrom(geometry_msgs::Point position, std::vector<unsigned char>& charmap_no_inflation);

    std::list<FrontierCell> searchFrontierBCell(std::vector<unsigned char> &charmap_no_inflation);

protected:

    /**
     * @brief Starting from an initial cell, build a frontier from valid adjacent cells
     * @param initial_cell Index of cell to start frontier building
     * @param reference Reference index to calculate position from
     * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
     * @return
     */
    Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag);
    Frontier buildNewFrontier(unsigned int initial_cell, std::vector<bool>& frontier_flag, std::vector<unsigned char> &charmap_no_inflation);
    Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag,
                              std::vector<unsigned char>& charmap_no_inflation);

    /**
     * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
     * @param idx Index of candidate cell
     * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
     * @return
     */
    bool isNewFrontierCell(unsigned int idx, unsigned int size_x, unsigned int size_y);
    bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag);
    bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag, std::vector<unsigned char>& charmap_no_inflation);

private:

    simple_map_2d::SimpleMap2D &simple_map_;
    unsigned char* map_;
    unsigned int size_x_ , size_y_;
    int min_frontier_size_;
    std::string travel_point_;

    ros::NodeHandle nh_;
    bool initial_goal_flag_;
    double wx_max, wx_min, wy_max, wy_min;

    double inflation_radius_;
    double resolution_;

};

}
#endif
