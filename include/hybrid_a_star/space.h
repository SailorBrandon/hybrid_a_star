#ifndef SPACE_H
#define SPACE_H

#include "node3d.h"
#include "constants.h"
#include <vector>
#include <nav_msgs/OccupancyGrid.h>

namespace hybrid_a_star
{
    class Space
    {
        public:
            Space(){};
            double getDeltaXY() const {return deltaXY_;}
            int getSize() const {return dimX_ * dimY_ * dimYaw_;}
            int getDimX() const {return dimX_;}
            int getDimY() const {return dimY_;}
            int getDimYaw() const {return dimYaw_;}
            bool isTraversable(const Node3D* node) const;
            void setMap(const nav_msgs::OccupancyGrid::Ptr map);

        private:
            std::vector<std::vector<int>> map_;
            int dimX_;
            int dimY_;
            int dimYaw_;
            double deltaXY_;
            int obstacleThreshold_ = 10; // TODO: make this a parameter
    };
}


#endif