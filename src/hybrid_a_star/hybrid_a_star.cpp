#include "hybrid_a_star/hybrid_a_star.h"

namespace hybrid_a_star
{
    bool HybridAStar::plan(PosePath &path, Space &space)
    {
        if (!path.endPtReady())
        {
            ROS_ERROR("Missing start pose!");
            return false;
        }
        ros::Time t0 = ros::Time::now();
        std::vector<Node3D> nodes3D(space.getSize());
        Node3D *nSoln = forwardSearch(path, space, nodes3D);
        if (nSoln == nullptr)
        {
            ROS_ERROR("No solution found!");
            return false;
        }
        std::vector<Node3D> nodePath;
        backTrack(nSoln, nodePath);
        path.updatePath(nodePath);
        ros::Time t1 = ros::Time::now();
        ros::Duration d(t1 - t0);
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

    Node3D *HybridAStar::forwardSearch(PosePath &path, Space &space, std::vector<Node3D> &nodes3D)
    {
        typedef boost::heap::binomial_heap<Node3D *,
                                           boost::heap::compare<CompareNodes>>
            priorityQueue;
        priorityQueue openSet;

        Node3D startNode(path.getStart()->pose.position.x,
                         path.getStart()->pose.position.y,
                         tf::getYaw(path.getStart()->pose.orientation));
        Node3D goalNode(path.getGoal()->pose.position.x,
                        path.getGoal()->pose.position.y,
                        tf::getYaw(path.getGoal()->pose.orientation));
        if (!space.isTraversable(&startNode) || !space.isTraversable(&goalNode))
        {
            ROS_WARN("Start or goal node is not traversable...");
            return nullptr;
        }

        // std::shared_ptr<Node3D> nPred;
        Node3D* nPred;
        int iPred = startNode.setIdx(space.getDimX(), space.getDimY(), space.getDimYaw(), space.getDeltaXY());
        nodes3D[iPred] = startNode;

        // std::shared_ptr<Node3D> nSucc;
        Node3D* nSucc;
        int iSucc;
        nodes3D[iPred].setOpen();
        openSet.push(&nodes3D[iPred]);

        int iter = 0;
        double newG;
        while (!openSet.empty())
        {
            ++iter;
            // nPred.reset(openSet.top());
            nPred = openSet.top();
            iPred = nPred->setIdx(space.getDimX(), space.getDimY(), space.getDimYaw(), space.getDeltaXY());
            openSet.pop();
            // Check for rewired node
            if (nodes3D[iPred].isClosed())
            {
                continue;
            }
            // Expanding the node
            else
            {
                nodes3D[iPred].setClosed();
                if (*nPred == goalNode || iter > Constants::maxIter)
                {
                    return nPred;
                }
                else
                {
                    // Search with Dubins Shot

                    // Search with forward simulation
                    for (int i = 0; i < Constants::numDir; ++i)
                    {
                        // nSucc.reset(nPred->getSucc(i));
                        nSucc = nPred->getSucc(i);
                        iSucc = nSucc->setIdx(space.getDimX(), space.getDimY(), space.getDimYaw(), space.getDeltaXY());
                        if (space.isTraversable(nSucc))
                        {
                            if (!nodes3D[iSucc].isClosed() || iPred == iSucc)
                            {
                                nSucc->updateG();
                                newG = nSucc->getG();
                                if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc)
                                {
                                    nSucc->updateH(&goalNode);
                                    // if the successor is in the same cell but the C value is larger
                                    if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker)
                                    {
                                        delete nSucc;
                                        continue;
                                    }
                                    // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                                    else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker)
                                    {
                                        nSucc->setPred(nPred->getPred());
                                    }

                                    if (nSucc->getPred() == nSucc)
                                    {
                                        std::cout << "looping";
                                    }

                                    nSucc->setOpen();
                                    nodes3D[iSucc] = *nSucc;
                                    std::cout << "pushing a new node" << std::endl;
                                    openSet.push(&nodes3D[iSucc]);
                                    delete nSucc;
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
                }
            }
        }
        return nullptr;
    }

    void HybridAStar::backTrack(const Node3D *nSoln, std::vector<Node3D> &nodePath)
    {
        nodePath.reserve(Constants::maxIter / 100);
        const Node3D *n = nSoln;
        while (n != nullptr)
        {
            nodePath.push_back(*n);
            n = n->getPred();
        }
        std::reverse(nodePath.begin(), nodePath.end());
        nodePath.shrink_to_fit();
    }
}
