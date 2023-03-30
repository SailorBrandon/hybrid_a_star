#include "hybrid_a_star/path.h"

namespace hybrid_a_star
{
    PosePath::PosePath(ros::NodeHandle &nh)
    {
        path_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);
        start_ = Node3D();
        goal_ = Node3D();
        hasGoal = false;
        hasStart = false;
    }

    void PosePath::setStart(const geometry_msgs::PoseStamped &start)
    {
        hasStart = true;
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/map", "/hy_map", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/map", "/hy_map", ros::Time(0), transform);
            double pose[3] = {start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation)};
            pose[0] -= transform.getOrigin().x();
            pose[1] -= transform.getOrigin().y();
            pose[0] = pose[0] * cos(transform.getRotation().getAngle()) + pose[1] * sin(transform.getRotation().getAngle());
            pose[1] = -pose[0] * sin(transform.getRotation().getAngle()) + pose[1] * cos(transform.getRotation().getAngle());
            pose[2] -= transform.getRotation().getAngle();
            start_.setX(pose[0]);
            start_.setY(pose[1]);
            start_.setYaw(pose[2]);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void PosePath::setGoal(const geometry_msgs::PoseStamped &goal)
    {
        hasGoal = true;
        geometry_msgs::PoseStamped goal_pose;
        listener.transformPose("/hy_map", ros::Time(0), goal, "/map", goal_pose);
        goal_.setX(goal_pose.pose.position.x);
        goal_.setY(goal_pose.pose.position.y);
        goal_.setYaw(tf::getYaw(goal_pose.pose.orientation));
    }

    void PosePath::backTrack(Node3D *nSoln, std::pair<int, int> pathLen)
    {
        nodePath_.clear();
        nodePath_.reserve(pathLen.first + pathLen.second);
        int count = pathLen.second;
        while (nSoln != nullptr)
        {
            nodePath_.push_back(*nSoln);
            Node3D *tmp = nSoln;
            nSoln = nSoln->getPred();
            if (count > 0)
            {
                delete tmp;
                count--;
            }
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
            if (node.isBackward())
                pose.pose.position.z = 0.5;
            else
                pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getYaw());
            listener.transformPose("/map", ros::Time(0), pose, "/hy_map", pose);
            pose.header.frame_id = "/map";
            path_msg.poses.push_back(pose);
        }
        path_pub_.publish(path_msg);
    }
}