#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H



#include <boost/heap/binomial_heap.hpp>

#include "hybrid_a_star/path.h"
#include "hybrid_a_star/space.h"
#include "hybrid_a_star/constants.h"
#include "hybrid_a_star/tf_broadcaster.h"

#include "hybrid_a_star/reeds_shepp.h"
#include <nav_msgs/OccupancyGrid.h>

namespace hybrid_a_star
{
    class HybridAStar
    {
    public:
        HybridAStar(): rs_planner_(Constants::minTurnR) {};
        bool plan(PosePath &path, Space &space);

    
    private:
        Node3D *forwardSearch(PosePath &path, Space &space, std::vector<Node3D> &nodes3D);
        void backTrack(const Node3D *nSoln, std::vector<Node3D> &nodePath);
        ReedsSheppStateSpace rs_planner_;
    };


}

#endif