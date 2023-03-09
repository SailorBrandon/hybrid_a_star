#include "hybrid_a_star/tf_broadcaster.h"

namespace hybrid_a_star
{
    HyTFBroadcaster::HyTFBroadcaster(ros::NodeHandle& nh)
    {
        map_sub_ = nh.subscribe("/map", 1, &HyTFBroadcaster::mapCallback, this);
        map_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("hy_map_pose", 1);
        timer_ = nh.createTimer(ros::Duration(0.1), &HyTFBroadcaster::timerCallback, this);
    }
    
    void HyTFBroadcaster::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        hyMapPose_.setOrigin(tf::Vector3(msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z));
        hyMapPose_.setRotation(tf::Quaternion(msg->info.origin.orientation.x, msg->info.origin.orientation.y, msg->info.origin.orientation.z, msg->info.origin.orientation.w));
        geometry_msgs::PoseStamped hyMapPoseMsg;
        tf::poseTFToMsg(hyMapPose_, hyMapPoseMsg.pose); 
        hyMapPoseMsg.header.frame_id = "hy_map";
        hyMapPoseMsg.header.stamp = ros::Time::now();
        map_pose_pub_.publish(hyMapPoseMsg);
    }
    
    void HyTFBroadcaster::timerCallback(const ros::TimerEvent& event)
    {
        broadcaster_.sendTransform(tf::StampedTransform(hyMapPose_, ros::Time::now(), "map", "hy_map"));
    }
}
