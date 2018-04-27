//
// Created by chengdaqian on 18-4-10.
//

#include <simple_map_2d/map_inflation.h>

namespace simple_map_2d
{
    MapInflation::MapInflation(SimpleMap2D* map_ptr)
            : inflation_radius_(0)
            , weight_(0)
            , inflate_unknown_(false)
            , cell_inflation_radius_(0)
            , cached_cell_inflation_radius_(0)
            , seen_(NULL)
            , cached_costs_(NULL)
            , cached_distances_(NULL)
            , map_ptr_(map_ptr)
    {
        inflation_access_ = new boost::recursive_mutex();

        boost::unique_lock<boost::recursive_mutex> lock(*inflation_access_);
        ros::NodeHandle nh("~/inflation"), g_nh;
        if (seen_)
            delete[] seen_;
        seen_ = nullptr;
        seen_size_ = 0;
        need_reinflation_ = false;


        resolution_ = map_ptr_->getResolution();
        cell_inflation_radius_ = cellDistance(inflation_radius_);
        computeCaches();

        unsigned int size_x = map_ptr_->getSizeInCellsX(), size_y = map_ptr_->getSizeInCellsY();
        if (seen_)
            delete[] seen_;
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];


        nh.param<double>("/inflation_radius", inflation_radius_, 0.5);
        cell_inflation_radius_ = cellDistance(inflation_radius_);
        inscribed_radius_ = cell_inflation_radius_;
        nh.param<double>("cost_scaling_factor", weight_, 10.0);
        need_reinflation_ = true;
        computeCaches();
    }

    void MapInflation::updateCosts(int min_i, int min_j, int max_i, int max_j)
    {
        boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
        if (cell_inflation_radius_ == 0)
            return;

        ROS_INFO("inflation flag 1");

        // make sure the inflation list is empty at the beginning of the cycle (should always be true)
        ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");

        unsigned char* master_array = map_ptr_->inflated_costmap_;
        unsigned int size_x = map_ptr_->getSizeInCellsX(), size_y = map_ptr_->getSizeInCellsY();

        //copy map data
        for (unsigned int idx = 0; idx < size_y * size_x; idx++){
            master_array[idx] = map_ptr_->getCost(idx);
            /*
            if (map_ptr_->getCost(idx) == LETHAL_OBSTACLE && master_array[idx] != LETHAL_OBSTACLE
                || map_ptr_->getCost(idx) != LETHAL_OBSTACLE && master_array[idx] == LETHAL_OBSTACLE)
                master_array[idx] = map_ptr_->getCost(idx);
                */
        }

        if (seen_ == NULL) {
            ROS_WARN("MapInflation::updateCosts(): seen_ array is NULL");
            seen_size_ = size_x * size_y;
            seen_ = new bool[seen_size_];
        }
        else if (seen_size_ != size_x * size_y)
        {
            ROS_WARN("MapInflation::updateCosts(): seen_ array size is wrong");
            delete[] seen_;
            seen_size_ = size_x * size_y;
            seen_ = new bool[seen_size_];
        }
        memset(seen_, false, size_x * size_y * sizeof(bool));

        // We need to include in the inflation cells outside the bounding
        // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
        // up to that distance outside the box can still influence the costs
        // stored in cells inside the box.
        min_i -= cell_inflation_radius_;
        min_j -= cell_inflation_radius_;
        max_i += cell_inflation_radius_;
        max_j += cell_inflation_radius_;

        min_i = std::max(0, min_i);
        min_j = std::max(0, min_j);
        max_i = std::min(int(size_x), max_i);
        max_j = std::min(int(size_y), max_j);

        // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
        // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

        // Start with lethal obstacles: by definition distance is 0.0
        std::vector<CellData>& obs_bin = inflation_cells_[0.0];
        for (int j = min_j; j < max_j; j++)
        {
            for (int i = min_i; i < max_i; i++)
            {
                int index = map_ptr_->getIndex(i, j);
                unsigned char cost = master_array[index];
                if (cost == LETHAL_OBSTACLE)
                {
                    obs_bin.push_back(CellData(index, i, j, i, j));
                }
            }
        }

        // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
        // can overtake previously inserted but farther away cells
        std::map<double, std::vector<CellData> >::iterator bin;
        for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
        {
            for (int i = 0; i < bin->second.size(); ++i)
            {
                // process all cells at distance dist_bin.first
                const CellData& cell = bin->second[i];

                unsigned int index = cell.index_;

                // ignore if already visited
                if (seen_[index])
                {
                    continue;
                }

                seen_[index] = true;

                unsigned int mx = cell.x_;
                unsigned int my = cell.y_;
                unsigned int sx = cell.src_x_;
                unsigned int sy = cell.src_y_;

                // assign the cost associated with the distance from an obstacle to the cell
                unsigned char cost = costLookup(mx, my, sx, sy);
                unsigned char old_cost = master_array[index];
                if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
                    master_array[index] = cost;
                else
                    master_array[index] = std::max(old_cost, cost);

                // attempt to put the neighbors of the current cell onto the inflation list
                if (mx > 0)
                    enqueue(index - 1, mx - 1, my, sx, sy);
                if (my > 0)
                    enqueue(index - size_x, mx, my - 1, sx, sy);
                if (mx < size_x - 1)
                    enqueue(index + 1, mx + 1, my, sx, sy);
                if (my < size_y - 1)
                    enqueue(index + size_x, mx, my + 1, sx, sy);
            }
        }

        inflation_cells_.clear();
    }

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
    inline void MapInflation::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                        unsigned int src_x, unsigned int src_y)
    {
        if (!seen_[index])
        {
            // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
            double distance = distanceLookup(mx, my, src_x, src_y);

            // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
            if (distance > cell_inflation_radius_)
                return;

            // push the cell data onto the inflation list and mark
            inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
        }
    }

    void MapInflation::computeCaches()
    {
        if (cell_inflation_radius_ == 0)
            return;

        // based on the inflation radius... compute distance and cost caches
        if (cell_inflation_radius_ != cached_cell_inflation_radius_)
        {
            deleteKernels();

            cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
            cached_distances_ = new double*[cell_inflation_radius_ + 2];

            for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
            {
                cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
                cached_distances_[i] = new double[cell_inflation_radius_ + 2];
                for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
                {
                    cached_distances_[i][j] = hypot(i, j);
                }
            }

            cached_cell_inflation_radius_ = cell_inflation_radius_;
        }

        for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
        {
            for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
            {
                cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
            }
        }
    }

    void MapInflation::deleteKernels()
    {
        if (cached_distances_ != NULL)
        {
            for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
            {
                if (cached_distances_[i])
                    delete[] cached_distances_[i];
            }
            if (cached_distances_)
                delete[] cached_distances_;
            cached_distances_ = NULL;
        }

        if (cached_costs_ != NULL)
        {
            for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
            {
                if (cached_costs_[i])
                    delete[] cached_costs_[i];
            }
            delete[] cached_costs_;
            cached_costs_ = NULL;
        }
    }

}
