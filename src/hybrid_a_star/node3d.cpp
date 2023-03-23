#include "hybrid_a_star/node3d.h"

namespace hybrid_a_star
{

    Node3D::Node3D(double x, double y, double yaw, double g, double h, Node3D *pred, NodeType type, int idx, int primIdx, bool backward)
        : x(x), y(y), yaw(yaw), g(g), h(h), pred(pred), type(type), idx(idx), primIdx(primIdx), backward(backward)
    {
    }

    Node3D::Node3D(const Node3D *other)
        : x(other->x), y(other->y), yaw(other->yaw), g(other->g), h(other->h), pred(other->pred), type(other->type), idx(other->idx), primIdx(other->primIdx), backward(other->backward)
    {
    }

    int Node3D::setIdx(int dimX, int dimY, int dimYaw, double deltaXY)
    {
        idx = static_cast<int>(x / deltaXY) * dimY * dimYaw +
              static_cast<int>(y / deltaXY) * dimYaw +
              static_cast<int>(yaw / Constants::deltaYawRad);
        return idx;
    }

    Node3D *Node3D::getSucc(const int i)
    {
        double xSucc;
        double ySucc;
        double yawSucc;
        // calculate successor positions forward
        if (i < 3)
        {
            xSucc = x + dx[i] * cos(yaw) - dy[i] * sin(yaw); 
            ySucc = y + dx[i] * sin(yaw) + dy[i] * cos(yaw);
            yawSucc = Utils::normalizeYawRad(yaw + dyaw[i]);
            return new Node3D(xSucc, ySucc, yawSucc, g, 0, this, UNDEF, -1, i, false);
        }
        // backwards
        else
        {
            xSucc = x - dx[i - 3] * cos(yaw) - dy[i - 3] * sin(yaw);
            ySucc = y - dx[i - 3] * sin(yaw) + dy[i - 3] * cos(yaw);
            yawSucc = Utils::normalizeYawRad(yaw - dyaw[i - 3]);
            return new Node3D(xSucc, ySucc, yawSucc, g, 0, this, UNDEF, -1, i, true);
        }
    }

    void Node3D::updateG()
    {
        // forward driving
        if (primIdx < 3)
        {
            // penalize turning
            if (pred->primIdx != primIdx)
            {
                // penalize change of direction
                if (pred->primIdx > 2)
                {
                    g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
                }
                else
                {
                    g += dx[0] * Constants::penaltyTurning;
                }
            }
            else
            {
                g += dx[0];
            }
        }
        // reverse driving
        else
        {
            // penalize turning and reversing
            if (pred->primIdx != primIdx)
            {
                // penalize change of direction
                if (pred->primIdx < 3)
                {
                    g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
                }
                else
                {
                    g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
                }
            }
            else
            {
                g += dx[0] * Constants::penaltyReversing;
            }
        }
    }    

    void Node3D::updateH(const Node3D *goal)
    {
        h = sqrt(pow(goal->getX() - x, 2) + pow(goal->getY() - y, 2)); // heuristic: euclidean distance
    }

    bool Node3D::operator==(const Node3D &rhs) const
    {
        return (std::abs(x - rhs.x) <= Constants::vehLength/10) &&
               (std::abs(y - rhs.y) <= Constants::vehLength/10) &&
               std::abs(yaw - rhs.yaw) <= Constants::deltaYawRad;
    }

}