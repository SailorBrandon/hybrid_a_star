#ifndef TF_BROADCASTER_H
#define TF_BROADCASTER_H

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

namespace hybrid_a_star
{
    class HyTFBroadcaster
    {
        public:
            HyTFBroadcaster(ros::NodeHandle& nh);
            void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
            void timerCallback(const ros::TimerEvent& event);
            
        
        private:
            tf::TransformBroadcaster broadcaster_;
            ros::Subscriber map_sub_;
            ros::Publisher map_pose_pub_;
            tf::Pose hyMapPose_; // the origin of the map
            ros::Timer timer_;
            bool mapReceived;
    };
}

#endif