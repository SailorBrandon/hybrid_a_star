#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include <unordered_map>
#include <vector>
#include <iostream>
#include <queue>

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
        HybridAStar() : rs_planner_(Constants::minTurnR){};
        bool plan(PosePath &path, Space &space);

    private:
        Node3D *forwardSearch(Node3D *startNode,
                              Node3D *goalNode,
                              Space &space,
                              std::pair<int, int> &pathLen,
                              std::unordered_map<int, Node3D *> &closedSet,
                              std::unordered_map<int, Node3D *> &openSet);
        void backTrack(const Node3D *nSoln, std::vector<Node3D> &nodePath);
        ReedsSheppStateSpace rs_planner_;
        Node3D *rsShot(Node3D *startNode, Node3D *goalNode, Space &space, std::pair<int, int> &pathLen);
    };

}

#endif