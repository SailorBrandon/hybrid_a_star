#ifndef PLANNER_MANAGER_H
#define PLANNER_MANAGER_H

#include <string>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "hybrid_a_star/hybrid_a_star.h"

namespace hybrid_a_star
{

  class PlannerManager : public nav_core::BaseGlobalPlanner
  {
  public:
    PlannerManager();
    PlannerManager(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    ~PlannerManager();

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);

  private:
    bool checkPose(const geometry_msgs::PoseStamped &input_pose);

    costmap_2d::Costmap2DROS *costmap_ros_;
    double step_size_;
    double min_dist_from_robot_;
    costmap_2d::Costmap2D *costmap_;
    base_local_planner::WorldModel *world_model_;
    bool initialized_;

    std::shared_ptr<HybridAStar> hybrid_a_star_;
  };
};

#endif