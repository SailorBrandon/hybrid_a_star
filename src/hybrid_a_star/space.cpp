#include "hybrid_a_star/space.h"


namespace hybrid_a_star
{
    void Space::setMap(const nav_msgs::OccupancyGrid::Ptr map)
    {
        dimX_ = map->info.width;
        dimY_ = map->info.height;
        dimYaw_ = Constants::dimYaw;
        deltaXY_ = map->info.resolution;
        map_.resize(dimX_);
        ROS_INFO("Map size: %d , %d", dimX_, dimY_);
        for (int i = 0; i < dimX_; i++)
        {
            map_[i].resize(dimY_);
            for (int j = 0; j < dimY_; j++)
            {
                map_[i][j] = map->data[i + j * dimX_];
            }
        }
    }

    bool Space::isTraversable(const Node3D* node) const
    {
        // check: 1) the node is within the map, 2) the node is not an obstacle
        int iX = static_cast<int>(node->getX() / deltaXY_);
        int iY = static_cast<int>(node->getY() / deltaXY_);
        int iYaw = static_cast<int>(node->getYaw() / Constants::deltaYawRad);
        return iX >= 0 && iX < dimX_ && iY >= 0 && iY < dimY_ && iYaw >= 0 && iYaw < dimYaw_ &&
               map_[iX][iY] >= 0 && map_[iX][iY] < obstacleThreshold_;
    }

    bool Space::isTraversable(const double x, const double y, const double yaw)
    {
        int iX = static_cast<int>(x / deltaXY_);
        int iY = static_cast<int>(y / deltaXY_);
        int iYaw = static_cast<int>(yaw / Constants::deltaYawRad);
        return iX >= 0 && iX < dimX_ && iY >= 0 && iY < dimY_ && iYaw >= 0 && iYaw < dimYaw_ &&
               map_[iX][iY] >= 0 && map_[iX][iY] < obstacleThreshold_;
    }
}