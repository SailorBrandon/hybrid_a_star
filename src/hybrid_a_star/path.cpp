#include "hybrid_a_star/path.h"

namespace hybrid_a_star
{
    PosePath::PosePath(ros::NodeHandle& nh)
    {
        path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
        start_ = Node3D();
        goal_ = Node3D();
        hasGoal = false;
        hasStart = false;
    }

    void PosePath::setStart(const geometry_msgs::PoseStamped& start)
    {
        hasStart = true;
        geometry_msgs::PoseStamped start_pose;
        listener.transformPose("/hy_map", ros::Time(0), start, "/map", start_pose);
        start_.setX(start_pose.pose.position.x);
        start_.setY(start_pose.pose.position.y);
        start_.setYaw(tf::getYaw(start_pose.pose.orientation));
    }

    void PosePath::setGoal(const geometry_msgs::PoseStamped& goal)
    {
        hasGoal = true;
        geometry_msgs::PoseStamped goal_pose;
        listener.transformPose("/hy_map", ros::Time(0), goal, "/map", goal_pose);
        goal_.setX(goal_pose.pose.position.x);
        goal_.setY(goal_pose.pose.position.y);
        goal_.setYaw(tf::getYaw(goal_pose.pose.orientation));
        // Debug
        // std::cout << "Goal before transform: " << goal.pose.position.x << ", " << goal.pose.position.y << std::endl;
        // std::cout << "Goal after transform: " << goal_->pose.position.x << ", " << goal_->pose.position.y << std::endl;
    }

    void PosePath::backTrack(const Node3D *nSoln, int pathLen)
    {
        nodePath_.clear();
        nodePath_.reserve(pathLen);
        const Node3D *n = nSoln;
        while (n != nullptr)
        {
            nodePath_.push_back(*n);
            n = n->getPred();
        }
        std::reverse(nodePath_.begin(), nodePath_.end());
    }

    void PosePath::pubPath()
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "/map";
        path_msg.header.stamp = ros::Time::now();
        path_msg.poses.reserve(nodePath_.size());
        for (auto node : nodePath_)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/hy_map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = node.getX();
            pose.pose.position.y = node.getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getYaw());
            listener.transformPose("/map", ros::Time(0), pose, "/hy_map", pose);
            pose.header.frame_id = "/map";
            path_msg.poses.push_back(pose);
        }
        path_pub_.publish(path_msg);
    }
}