#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "hybrid_a_star/hybrid_a_star.h"

using namespace hybrid_a_star;

class HybridAStarNode
{
    public:
        explicit HybridAStarNode(ros::NodeHandle& nh) // TODO: why not const?
            : path_(nh)
        {
            tf_broadcaster_ = std::make_shared<HyTFBroadcaster>(nh);
            hybrid_a_star_ = std::make_shared<HybridAStar>();
            map_sub_ = nh.subscribe("/map", 1, &HybridAStarNode::setMap, this);
            goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &HybridAStarNode::setGoal, this);
            init_pose_sub_ = nh.subscribe("/initialpose", 1, &HybridAStarNode::setInitPose, this);
            obsv_sub_ = nh.subscribe("/odom", 1, &HybridAStarNode::setOdom, this);
        }

        void setMap(const nav_msgs::OccupancyGrid::Ptr map)
        {
            ROS_INFO("Map received");
            space_.setMap(map);
        }

        void setGoal(const geometry_msgs::PoseStamped::Ptr goal)
        {
            ROS_INFO("Goal received");
            path_.setGoal(*goal);
            if (path_.ready() && hybrid_a_star_->plan(path_, space_))
            {
                path_.pubPath();
            }
        }

        void setInitPose(const geometry_msgs::PoseWithCovarianceStamped::Ptr start)
        {
            ROS_INFO("Init pose received");
            geometry_msgs::PoseStamped::Ptr start_pose = boost::make_shared<geometry_msgs::PoseStamped>();
            start_pose->header = start->header;
            start_pose->pose = start->pose.pose;
            path_.setStart(*start_pose);
            if (path_.ready() && hybrid_a_star_->plan(path_, space_))
            {
                path_.pubPath();
            }
        }

        void setOdom(const nav_msgs::Odometry::Ptr odom)
        {
            // ROS_INFO("Odom received");
            geometry_msgs::PoseStamped::Ptr start_pose = boost::make_shared<geometry_msgs::PoseStamped>();
            start_pose->header = odom->header;
            start_pose->pose = odom->pose.pose;
            path_.setStart(*start_pose);
        }


    private:
        ros::Subscriber map_sub_;
        ros::Subscriber goal_sub_;
        ros::Subscriber init_pose_sub_;
        ros::Subscriber obsv_sub_;
        std::shared_ptr<HybridAStar> hybrid_a_star_;
        Space space_;
        PosePath path_;
        std::shared_ptr<HyTFBroadcaster> tf_broadcaster_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hybrid_a_star_node");
    ros::NodeHandle nh;
    HybridAStarNode hybrid_a_star_node(nh);
    ros::spin();
    return 0;
}