#ifndef PATH_H
#define PATH_H

#include <vector>
#include <iostream>
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
            void updatePath(const std::vector<Node3D> &nodePath);
            bool ready();
            void pubPath();
            const geometry_msgs::PoseStamped::Ptr& getStart() const {return start_;}
            const geometry_msgs::PoseStamped::Ptr& getGoal() const {return goal_;}

        private:
            ros::Publisher path_pub_;
            geometry_msgs::PoseStamped::Ptr start_;
            geometry_msgs::PoseStamped::Ptr goal_;
            nav_msgs::Path path_;
            tf::TransformListener listener;
            bool hasGoal;
            bool hasStart;
    };
}

#endif