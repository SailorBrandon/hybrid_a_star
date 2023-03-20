#include "hybrid_a_star/path.h"

namespace hybrid_a_star
{
    PosePath::PosePath(ros::NodeHandle& nh)
    {
        path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
        start_ = boost::make_shared<geometry_msgs::PoseStamped>();
        goal_ = boost::make_shared<geometry_msgs::PoseStamped>();
        hasGoal = false;
        hasStart = false;
    }

    void PosePath::setStart(const geometry_msgs::PoseStamped& start)
    {
        hasStart = true;
        start_->header.frame_id = "/map";
        listener.transformPose("/hy_map", ros::Time(0), start, "/map", *start_);
    }

    void PosePath::setGoal(const geometry_msgs::PoseStamped& goal)
    {
        hasGoal = true;
        goal_->header.frame_id = "/map";
        listener.transformPose("/hy_map", ros::Time(0), goal, "/map", *goal_);
        // Debug
        // std::cout << "Goal before transform: " << goal.pose.position.x << ", " << goal.pose.position.y << std::endl;
        // std::cout << "Goal after transform: " << goal_->pose.position.x << ", " << goal_->pose.position.y << std::endl;
    }

    void PosePath::updatePath(const std::vector<Node3D> &nodePath)
    {
        path_.poses.clear();
        path_.header.frame_id = "/map";
        path_.header.stamp = ros::Time::now();
        path_.poses.reserve(nodePath.size());
        for (auto node : nodePath)
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
            path_.poses.push_back(pose);
        }
    }

    bool PosePath::ready()
    {
        return hasStart && hasGoal;
    }

    void PosePath::pubPath()
    {
        path_pub_.publish(path_);
    }
}