#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

#include <boost/heap/binomial_heap.hpp>

#include "hybrid_a_star/path.h"
#include "hybrid_a_star/space.h"
#include "hybrid_a_star/constants.h"
#include "hybrid_a_star/tf_broadcaster.h"

#include <nav_msgs/OccupancyGrid.h>

namespace hybrid_a_star
{
    class HybridAStar
    {
    public:
        HybridAStar(){};
        bool plan(PosePath &path, Space &space);
        Node3D *forwardSearch(PosePath &path, Space &space, std::vector<Node3D> &nodes3D);
        void backTrack(const Node3D *nSoln, std::vector<Node3D> &nodePath);
    };
}

#endif