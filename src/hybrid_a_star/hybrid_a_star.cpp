#include "hybrid_a_star/hybrid_a_star.h"

namespace hybrid_a_star
{
    bool HybridAStar::plan(PosePath &path, Space &space)
    {
        if (!path.ready())
        {
            ROS_ERROR("Missing start or goal!");
            return false;
        }
        ros::Time t0 = ros::Time::now();
        std::pair<int, int> pathLen;
        Node3D* startPtr = new Node3D(path.getStart());
        Node3D* goalPtr = new Node3D(path.getGoal());
        std::unordered_map<int, Node3D *> closedSet;
        std::unordered_map<int, Node3D *> openSet;
        Node3D *nSoln = forwardSearch(startPtr, goalPtr, space, pathLen, closedSet, openSet);
        if (nSoln == nullptr)
        {
            ROS_ERROR("No solution found!");
            ros::Time t1 = ros::Time::now();
            ros::Duration d = t1 - t0;
            ROS_INFO("TIME in ms: %f", d.toSec() * 1000);   
            return false;
        }
        path.backTrack(nSoln, pathLen);
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
        // smoother_.smooth(path);
        ros::Time t1 = ros::Time::now();
        ros::Duration d = t1 - t0;
        ROS_INFO("TIME in ms: %f", d.toSec() * 1000);        
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
                                       Space &space,
                                       std::pair<int, int> &pathLen,
                                       std::unordered_map<int, Node3D *> &closedSet,
                                       std::unordered_map<int, Node3D *> &openSet)
    {

        if (!space.isTraversable(startPtr) || !space.isTraversable(goalPtr))
        {
            ROS_WARN("Start or goal node is not traversable...");
            return nullptr;
        }

        std::priority_queue<Node3D *, std::vector<Node3D *>, CompareNodes> priQue;

        Node3D *nPred;
        int iPred;
        Node3D *nSucc;
        int iSucc;

        openSet[startPtr->setIdx(space.getDimX(), space.getDimY(), space.getDimYaw(), space.getDeltaXY())] = startPtr;
        priQue.push(startPtr);

        int iter = 0;
        while (!priQue.empty() && iter < Constants::maxIter)
        {
            ++iter;
            nPred = priQue.top();
            iPred = nPred->setIdx(space.getDimX(), space.getDimY(), space.getDimYaw(), space.getDeltaXY());
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
                Node3D *nSoln = rsShot(nPred, goalPtr, space, pathLen);
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
                iSucc = nSucc->setIdx(space.getDimX(), space.getDimY(), space.getDimYaw(), space.getDeltaXY());
                if (space.isTraversable(nSucc))
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

    Node3D *HybridAStar::rsShot(Node3D *startPtr, Node3D *goalPtr, Space &space, std::pair<int, int> &pathLen)
    {
        double q0[3] = {startPtr->getX(), startPtr->getY(), startPtr->getYaw()};
        double q1[3] = {goalPtr->getX(), goalPtr->getY(), goalPtr->getYaw()};
        std::vector<std::vector<double>> rs_path;
        rs_planner_.sample(q0, q1, Constants::rsStepSize, rs_path);
        for (auto point : rs_path)
        {
            if (!space.isTraversable(point[0], point[1], point[2]))
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
}
