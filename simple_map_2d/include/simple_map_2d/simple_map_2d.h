#ifndef SIMPLE_MAP_2D_H_
#define SIMPLE_MAP_2D_H_

#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>

namespace simple_map_2d
{
    static const unsigned char NO_INFORMATION = 255;
    static const unsigned char LETHAL_OBSTACLE = 254;
    static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    static const unsigned char FREE_SPACE = 0;

    static const unsigned char laser_type = 0;
    static const unsigned char point_type = 1;

    struct MapLocation
    {
        unsigned int x;
        unsigned int y;
    };

    struct SensorInput
    {
        std::string topic_name;
        unsigned char sensor_type;
        std::string frame_id;
    };

    class SimpleMap2D
    {

    public:
        typedef boost::recursive_mutex mutex_t;
        SimpleMap2D(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                    double origin_x, double origin_y, unsigned char default_value = 0);
        SimpleMap2D(); // default constructer

        ~SimpleMap2D();

        unsigned char getCost(unsigned int mx, unsigned int my) const;
        unsigned char getCost(unsigned int idx) const;
        unsigned char getCostInflated(unsigned int mx, unsigned int my) const;
        unsigned char getCostInflated(unsigned int idx) const;

        double getOriginX() const{
            return origin_x_;
        }

        double getOriginY() const{
            return origin_y_;
        }

        void setCost(unsigned int mx, unsigned int my, unsigned char cost);
        void setCost(unsigned int idx, unsigned char cost);
        void setCostInflated(unsigned int mx, unsigned int my, unsigned char cost);
        void setCostInflated(unsigned int idx, unsigned char cost);

        void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);

        inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const {
            my = index / size_x_;
            mx = index - (my * size_x_);
        }

        inline unsigned int getIndex(unsigned int mx, unsigned int my) const
        {
            return my * size_x_ + mx;
        }

        unsigned int getSizeInCellsX() const { return size_x_; }
        unsigned int getSizeInCellsY() const { return size_y_; }
        double getResolution() const { return resolution_; }

        bool setPolygonEdgeCost(const std::vector<geometry_msgs::Point>& polygon, unsigned char cost_value);

        unsigned int cellDistance(double world_dist);

        inline void raytraceLineKeepObstacle(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                                             unsigned int max_length = UINT_MAX){
            int dx = x1 - x0;
            int dy = y1 - y0;

            unsigned int abs_dx = abs(dx);
            unsigned int abs_dy = abs(dy);

            int offset_dx = sign(dx);
            int offset_dy = sign(dy) * size_x_;

            unsigned int offset = y0 * size_x_ + x0;

            // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
            double dist = hypot(dx, dy);
            double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

            // if x is dominant
            if (abs_dx >= abs_dy)
            {
                int error_y = abs_dx / 2;
                bresenham2DKeepObstacle(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
                return;
            }

            // otherwise y is dominant
            int error_x = abs_dy / 2;
            bresenham2DKeepObstacle(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
        }

        inline void raytraceLine(unsigned char cost_value, unsigned int x0, unsigned int y0,
                                 unsigned int x1, unsigned int y1, unsigned int max_length = UINT_MAX){

            if (x0 > size_x_ || x1 > size_x_ || y0 > size_y_ || y1 > size_y_)
                return;

            int dx = x1 - x0;
            int dy = y1 - y0;

            unsigned int abs_dx = abs(dx);
            unsigned int abs_dy = abs(dy);

            int offset_dx = sign(dx);
            int offset_dy = sign(dy) * size_x_;

            unsigned int offset = y0 * size_x_ + x0;

            // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
            double dist = hypot(dx, dy);
            double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

            // if x is dominant
            if (abs_dx >= abs_dy)
            {
                int error_y = abs_dx / 2;
                bresenham2D(cost_value, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
                return;
            }

            // otherwise y is dominant
            int error_x = abs_dy / 2;
            bresenham2D(cost_value, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
        }

        inline bool checkRaytraceFree(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1){
            int dx = x1 - x0;
            int dy = y1 - y0;

            unsigned int abs_dx = abs(dx); // abs delta x
            unsigned int abs_dy = abs(dy); // abs delta y

            unsigned int size_x = size_x_;

            int offset_dx = (dx > 0 ? 1 : -1); // dx in map idx
            int offset_dy = (dy > 0 ? 1 : -1) * size_x; // dy in map idx

            unsigned int max_length = std::numeric_limits<unsigned int>::max();

            unsigned int offset = y0 * size_x + x0; // point0 idx

            double dist = hypot(dx, dy); // 2D distance
            double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

            if (abs_dx >= abs_dy)
            {
                int error_y = abs_dx / 2;
                return bresenham2D_raytraceFree(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
            }else{
                int error_x = abs_dy / 2;
                return bresenham2D_raytraceFree(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
            }
        }

        boost::recursive_mutex* getMutex(){
            return access_;
        }

        unsigned char* inflated_costmap_;

        //void copyAndInflate();

    private:
        void initMaps(unsigned int size_x, unsigned int size_y);

        void deleteMap();

        //virtual void raytraceFreespaceKeepObstacles(const costmap_2d::Observation& clearing_observation,
        //                               double* min_x, double* min_y, double* max_x, double* max_y);



        inline void bresenham2DKeepObstacle(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                                            int offset_b, unsigned int offset, unsigned int max_length){
            unsigned int end = std::min(max_length, abs_da);
            for (unsigned int i = 0; i < end; ++i)
            {
                setValueKeepObstacle(offset, FREE_SPACE);
                offset += offset_a;
                error_b += abs_db;
                if ((unsigned int)error_b >= abs_da)
                {
                    offset += offset_b;
                    error_b -= abs_da;
                }
            }
            setValueKeepObstacle(offset, FREE_SPACE);
        }

        inline void setValueKeepObstacle(unsigned int idx, unsigned char value){
            if (costmap_[idx] != LETHAL_OBSTACLE){
                costmap_[idx] = value;
            }
        }

        inline void bresenham2D(unsigned char cost_value, unsigned int abs_da, unsigned int abs_db, int error_b,
                                int offset_a, int offset_b, unsigned int offset, unsigned int max_length)
        {
            unsigned int end = std::min(max_length, abs_da);
            for (unsigned int i = 0; i < end; ++i)
            {
                costmap_[offset] = cost_value;
                offset += offset_a;
                error_b += abs_db;
                if ((unsigned int)error_b >= abs_da)
                {
                    offset += offset_b;
                    error_b -= abs_da;
                }
            }
            costmap_[offset] = cost_value;
        }

        inline bool bresenham2D_raytraceFree(unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, unsigned int offset, unsigned int max_length){

            unsigned int end = std::min(max_length, abs_da);

            for (unsigned int i = 0; i < end; ++i)
            {
                if (getCost(offset) != FREE_SPACE && getCost(offset) != NO_INFORMATION)
                {
                    return false;
                }
                offset += offset_a;
                error_b += abs_db;
                if ((unsigned int)error_b >= abs_da)
                {
                    offset += offset_b;
                    error_b -= abs_da;
                }
            }

            if (getCost(offset) != FREE_SPACE && getCost(offset) != NO_INFORMATION){
                return false;
            } else{
                return true;
            }
        }

        inline int sign(int x)
        {
            return x > 0 ? 1.0 : -1.0;
        }

    //private:
    public:
        boost::recursive_mutex* access_;
        unsigned int size_x_;
        unsigned int size_y_;
        double resolution_;
        double origin_x_;
        double origin_y_;
        unsigned char* costmap_;
        unsigned char default_value_;

        bool* seen_;
        int seen_size_;

        //MapInflation *inflation_layer_;

    };
}

#endif