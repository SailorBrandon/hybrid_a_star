#ifndef PATH_H
#define PATH_H

#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "hybrid_a_star/node3d.h"

namespace hybrid_a_star
{
    class PosePath
    {
        public:
            PosePath(ros::NodeHandle& nh);
            void setStart(const geometry_msgs::PoseStamped& start);
            void setGoal(const geometry_msgs::PoseStamped& goal);
            Node3D* getStart() { return &start_; }
            Node3D* getGoal() { return &goal_; }
            bool ready() const { return hasStart && hasGoal; }
            void backTrack(Node3D *nSoln, std::pair<int, int> pathLen);
            void pubPath();
        
        private:
            ros::Publisher path_pub_;
            Node3D start_;
            Node3D goal_;
            std::vector<Node3D> nodePath_;
            tf::TransformListener listener;
            bool hasGoal;
            bool hasStart;
    };
}

#endif