#include "hybrid_a_star/hybrid_a_star.h"

namespace hybrid_a_star
{
    bool HybridAStar::getPlan(const geometry_msgs::PoseStamped &start,
                              const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan,
                              costmap_2d::Costmap2D *costmap)
    {
        ros::Time t0 = ros::Time::now();
        std::pair<int, int> pathLen;
        Node3D *startPtr = new Node3D(start.pose.position.x, start.pose.position.y, tf::getYaw(start.pose.orientation));
        Node3D *goalPtr = new Node3D(goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));
        std::unordered_map<int, Node3D *> closedSet;
        std::unordered_map<int, Node3D *> openSet;
        Node3D *nSoln = forwardSearch(startPtr, goalPtr, costmap, pathLen, closedSet, openSet);
        if (nSoln == nullptr)
        {
            ROS_ERROR("No solution found!");
            ros::Time t1 = ros::Time::now();
            ros::Duration d = t1 - t0;
            ROS_INFO("TIME in ms: %f", d.toSec() * 1000);
            return false;
        }
        backTrack(nSoln, pathLen, plan);
        for (auto &node : closedSet)
        {
            // std::cout << "closedSet: " << node.first << std::endl;
            delete node.second;
        }
        for (auto &node : openSet)
        {
            // TODO: elements in openSet and closedSet are the same
            // std::cout << "openSet: " << node.first << std::endl;
            if (closedSet.find(node.first) == closedSet.end())
                delete node.second;
        }
        return true;
    }

    struct CompareNodes
    {
        /// Sorting 3D nodes by increasing C value - the total estimated cost
        bool operator()(const Node3D *lhs, const Node3D *rhs) const
        {
            return lhs->getC() > rhs->getC();
        }
    };

    Node3D *HybridAStar::forwardSearch(Node3D *startPtr,
                                       Node3D *goalPtr,
                                       costmap_2d::Costmap2D *costmap,
                                       std::pair<int, int> &pathLen,
                                       std::unordered_map<int, Node3D *> &closedSet,
                                       std::unordered_map<int, Node3D *> &openSet)
    {
        std::priority_queue<Node3D *, std::vector<Node3D *>, CompareNodes> priQue;

        Node3D *nPred;
        int iPred;
        Node3D *nSucc;
        int iSucc;

        openSet[startPtr->setIdx(costmap->getSizeInCellsX(),
                                 costmap->getSizeInCellsY(),
                                 72,
                                 costmap->getResolution())] = startPtr;
        priQue.push(startPtr);

        int iter = 0;
        while (!priQue.empty() && iter < Constants::maxIter)
        {
            ++iter;
            nPred = priQue.top();
            iPred = nPred->setIdx(costmap->getSizeInCellsX(),
                                  costmap->getSizeInCellsY(),
                                  72,
                                  costmap->getResolution());
            closedSet[iPred] = nPred;
            priQue.pop();
            if (*nPred == *goalPtr)
            {
                pathLen.first = iter;
                ROS_INFO("Found a plan by forward search (%d)", pathLen.first);
                return nPred;
            }
            // finding solution by reeds-shepps shot
            if (iter % Constants::rsShotPeriod == 0)
            {
                Node3D *nSoln = rsShot(nPred, goalPtr, costmap, pathLen);
                if (nSoln != nullptr)
                {
                    pathLen.first = iter;
                    ROS_INFO("Found a plan by forward search (%d), RS shot (%d)", pathLen.first, pathLen.second);
                    return nSoln;
                }
            }
            //  finding solution by forward simulation
            for (int i = 0; i < Constants::numDir; ++i)
            {
                nSucc = nPred->getSucc(i);
                iSucc = nSucc->setIdx(costmap->getSizeInCellsX(),
                                      costmap->getSizeInCellsY(),
                                      72,
                                      costmap->getResolution());
                if (isTraversable(nSucc, costmap))
                {
                    if (closedSet.find(iSucc) == closedSet.end())
                    {
                        nSucc->updateG();
                        if (openSet.find(iSucc) == openSet.end() || nSucc->getG() < openSet[iSucc]->getG())
                        {
                            nSucc->updateH(goalPtr);
                            openSet[iSucc] = nSucc;
                            priQue.push(nSucc);
                        }
                        else
                        {
                            delete nSucc;
                        }
                    }
                    else
                    {
                        delete nSucc;
                    }
                }
                else
                {
                    delete nSucc;
                }
            }
        }
        return nullptr;
    }

    Node3D *HybridAStar::rsShot(Node3D *startPtr, Node3D *goalPtr, costmap_2d::Costmap2D *costmap, std::pair<int, int> &pathLen)
    {
        double q0[3] = {startPtr->getX(), startPtr->getY(), startPtr->getYaw()};
        double q1[3] = {goalPtr->getX(), goalPtr->getY(), goalPtr->getYaw()};
        std::vector<std::vector<double>> rs_path;
        rs_planner_.sample(q0, q1, Constants::rsStepSize, rs_path);
        for (auto point : rs_path)
        {
            if (!isTraversable(point[0], point[1], point[2], costmap))
            {
                return nullptr;
            }
        }
        pathLen.second = rs_path.size() - 1;
        for (int i = 1; i < rs_path.size(); ++i)
        {
            Node3D *n = new Node3D(rs_path[i][0], rs_path[i][1], rs_path[i][2]);
            n->setBackward(rs_path[i][4] < 0);
            n->setPred(startPtr);
            startPtr = n;
        }
        return startPtr;
    }

    void HybridAStar::backTrack(Node3D *nSoln,
                                std::pair<int, int> pathLen,
                                std::vector<geometry_msgs::PoseStamped> &plan)
    {
        std::vector<Node3D> nodePath;
        nodePath.reserve(pathLen.first + pathLen.second);
        int count = pathLen.second;
        while (nSoln != nullptr)
        {
            nodePath.push_back(*nSoln);
            Node3D *tmp = nSoln;
            nSoln = nSoln->getPred();
            if (count > 0)
            {
                delete tmp;
                count--;
            }
        }
        std::reverse(nodePath.begin(), nodePath.end());
        plan.resize(nodePath.size());
        for (int i = 0; i < nodePath.size(); ++i)
        {
            plan[i].header.frame_id = frame_id_;
            plan[i].header.stamp = ros::Time::now();
            plan[i].pose.position.x = nodePath[i].getX();
            plan[i].pose.position.y = nodePath[i].getY();
            // if (nodePath[i].isBackward())
            //     plan[i].pose.position.z = 0.5;
            // else
            //     plan[i].pose.position.z = 0.0;
            plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(nodePath[i].getYaw());
        }
    }

    bool HybridAStar::isTraversable(Node3D *node, costmap_2d::Costmap2D *costmap)
    {
        return isTraversable(node->getX(), node->getY(), node->getYaw(), costmap);
    }

    bool HybridAStar::isTraversable(double x, double y, double yaw, costmap_2d::Costmap2D *costmap)
    {
        double x0 = x - Constants::vehLength / 2 * cos(yaw);
        double y0 = y - Constants::vehLength / 2 * sin(yaw);
        double x1 = x + Constants::vehLength / 2 * cos(yaw);
        double y1 = y + Constants::vehLength / 2 * sin(yaw);
        unsigned int front_x, front_y, rear_x, rear_y;
        costmap->worldToMap(x0, y0, front_x, front_y);
        costmap->worldToMap(x1, y1, rear_x, rear_y);
        return (costmap->getCost(front_x, front_y) < costmap_2d::LETHAL_OBSTACLE) &&
               (costmap->getCost(rear_x, rear_y) < costmap_2d::LETHAL_OBSTACLE);
    }
}
