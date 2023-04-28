#include <pluginlib/class_list_macros.h>
#include "hybrid_a_star/planner_manager.h"

PLUGINLIB_EXPORT_CLASS(hybrid_a_star::PlannerManager, nav_core::BaseGlobalPlanner)

namespace hybrid_a_star
{
    PlannerManager::PlannerManager()
    {
    }

    PlannerManager::PlannerManager(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void PlannerManager::initialize(std::string name,
                                    costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;            // initialize the costmap_ros_ attribute to the parameter.
            costmap_ = costmap_ros_->getCostmap(); // get the costmap_ from costmap_ros_

            // initialize other planner parameters
            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
            world_model_ = new base_local_planner::CostmapModel(*costmap_); 

            hybrid_a_star_ = std::make_shared<HybridAStar>(costmap_ros->getGlobalFrameID());
            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }

    bool PlannerManager::makePlan(const geometry_msgs::PoseStamped &start,
                                  const geometry_msgs::PoseStamped &goal,
                                  std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!(checkPose(start) && checkPose(goal)))
            return false;
        if (hybrid_a_star_->getPlan(start, goal, plan, costmap_))
            return true;
        return false;
    }

    bool PlannerManager::checkPose(const geometry_msgs::PoseStamped &input_pose)
    {
        unsigned int pose_x, pose_y;
        if (costmap_->worldToMap(input_pose.pose.position.x, input_pose.pose.position.y, pose_x, pose_y) == 0)
        {
            ROS_WARN("The input pose is outside the costmap. Planning will always fail to this goal.");
            return false;
        }
        else
        {
            if (costmap_->getCost(pose_x, pose_y) == costmap_2d::LETHAL_OBSTACLE)
            {
                ROS_WARN("The input pose is in a lethal obstacle. Planning will always fail to this goal.");
                return false;
            }
        }
        return true;
    }

    PlannerManager::~PlannerManager()
    {
        delete world_model_;
    }
}
