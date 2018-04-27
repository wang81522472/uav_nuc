//
// Created by chengdaqian on 18-4-10.
//

#ifndef SIMPLE_MAP_2D_MAP_INFLATION_H
#define SIMPLE_MAP_2D_MAP_INFLATION_H
#include <ros/ros.h>
#include <simple_map_2d/simple_map_2d.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

namespace simple_map_2d
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
    class CellData
    {
    public:
        /**
         * @brief  Constructor for a CellData objects
         * @param  i The index of the cell in the cost map
         * @param  x The x coordinate of the cell in the cost map
         * @param  y The y coordinate of the cell in the cost map
         * @param  sx The x coordinate of the closest obstacle cell in the costmap
         * @param  sy The y coordinate of the closest obstacle cell in the costmap
         * @return
         */
        CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
                index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
        {
        }
        unsigned int index_;
        unsigned int x_, y_;
        unsigned int src_x_, src_y_;
    };

    class MapInflation
    {
    public:
        MapInflation(SimpleMap2D* map_ptr);

        ~MapInflation()
        {
            deleteKernels();
        }

        void updateCosts(int min_i, int min_j, int max_i, int max_j);

        /** @brief  Given a distance, compute a cost.
         * @param  distance The distance from an obstacle in cells
         * @return A cost value for the distance */
        inline unsigned char computeCost(double distance) const
        {
            unsigned char cost = 0;
            if (distance == 0)
                cost = LETHAL_OBSTACLE;
            else if (distance * resolution_ <= inscribed_radius_)
                cost = INSCRIBED_INFLATED_OBSTACLE;
            else
            {
                // make sure cost falls off by Euclidean distance
                double euclidean_distance = distance * resolution_;
                double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
                cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
            }
            return cost;
        }

    protected:
        boost::recursive_mutex* inflation_access_;

    private:
        /**
         * @brief  Lookup pre-computed distances
         * @param mx The x coordinate of the current cell
         * @param my The y coordinate of the current cell
         * @param src_x The x coordinate of the source cell
         * @param src_y The y coordinate of the source cell
         * @return
         */
        inline double distanceLookup(int mx, int my, int src_x, int src_y)
        {
            unsigned int dx = abs(mx - src_x);
            unsigned int dy = abs(my - src_y);
            return cached_distances_[dx][dy];
        }

        /**
         * @brief  Lookup pre-computed costs
         * @param mx The x coordinate of the current cell
         * @param my The y coordinate of the current cell
         * @param src_x The x coordinate of the source cell
         * @param src_y The y coordinate of the source cell
         * @return
         */
        inline unsigned char costLookup(int mx, int my, int src_x, int src_y)
        {
            unsigned int dx = abs(mx - src_x);
            unsigned int dy = abs(my - src_y);
            return cached_costs_[dx][dy];
        }

        void computeCaches();
        void deleteKernels();
        //void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);

        unsigned int cellDistance(double world_dist)
        {
            return map_ptr_->cellDistance(world_dist);
        }

        inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                            unsigned int src_x, unsigned int src_y);

        double inflation_radius_, inscribed_radius_, weight_;
        bool inflate_unknown_;
        unsigned int cell_inflation_radius_;
        unsigned int cached_cell_inflation_radius_;
        std::map<double, std::vector<CellData> > inflation_cells_;

        double resolution_;

        bool* seen_;
        int seen_size_;

        unsigned char** cached_costs_;
        double** cached_distances_;

        bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.

    public:
        SimpleMap2D* map_ptr_;
    };

}

#endif //SIMPLE_MAP_2D_MAP_INFLATION_H
