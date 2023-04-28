#ifndef HYBRID_A_STAR_H
#define HYBRID_A_STAR_H

#include <unordered_map>
#include <vector>
#include <iostream>
#include <queue>

#include <ros/ros.h>
#include <tf/tf.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

#include "hybrid_a_star/node3d.h"
#include "hybrid_a_star/constants.h"
#include "hybrid_a_star/reeds_shepp.h"

namespace hybrid_a_star
{
    class HybridAStar
    {
    public:
        HybridAStar(std::string frame_id) : rs_planner_(Constants::minTurnR), frame_id_(frame_id){};
        bool getPlan(const geometry_msgs::PoseStamped &start,
                     const geometry_msgs::PoseStamped &goal,
                     std::vector<geometry_msgs::PoseStamped> &plan,
                     costmap_2d::Costmap2D *costmap);

    private:
        Node3D *forwardSearch(Node3D *startPtr,
                              Node3D *goalPtr,
                              costmap_2d::Costmap2D *costmap,
                              std::pair<int, int> &pathLen,
                              std::unordered_map<int, Node3D *> &closedSet,
                              std::unordered_map<int, Node3D *> &openSet);
        ReedsSheppStateSpace rs_planner_;
        Node3D *rsShot(Node3D *startPtr, Node3D *goalPtr, costmap_2d::Costmap2D *costmap, std::pair<int, int> &pathLen);
        void backTrack(Node3D *nSoln,
                       std::pair<int, int> pathLen,
                       std::vector<geometry_msgs::PoseStamped> &plan);
        bool isTraversable(Node3D *node, costmap_2d::Costmap2D *costmap);
        bool isTraversable(double x, double y, double yaw, costmap_2d::Costmap2D *costmap);
        std::string frame_id_;
    };

}

#endif