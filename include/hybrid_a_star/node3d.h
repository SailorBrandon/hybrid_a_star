#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>
#include <vector>
#include "hybrid_a_star/utils.h"
#include "hybrid_a_star/constants.h"

namespace hybrid_a_star
{
    enum NodeType
    {
        OPEN,
        CLOSED,
        UNDEF
    };

    class Node3D
    {
        public:
            Node3D(): Node3D(0, 0, 0, 0, 0, nullptr, UNDEF, 0, 0) {};
            Node3D(double x, double y, double yaw): Node3D(x, y, yaw, 0, 0, nullptr, UNDEF, 0, 0) {};
            Node3D(double x, double y, double yaw, double g, double h, const Node3D* pred, NodeType type, int idx, int primIdx);
            double getX() const {return x;}
            double getY() const {return y;}
            double getYaw() const {return yaw;}
            double getG() const {return g;}
            double getC() const {return g + h;}
            void setClosed() {type = CLOSED;}
            void setOpen() {type = OPEN;}
            bool isClosed() const {return type == CLOSED;}
            bool isOpen() const {return type == OPEN;}
            const Node3D* getPred() const {return pred;}
            Node3D* getSucc(const int i) const;
            void setPred(const Node3D* pred) {this->pred = pred;}
            int setIdx(int dimX, int dimY, int dimYaw, double deltaXY);
            int getIdx() const {return idx;}
            void updateG();
            void updateH(const Node3D* goal);
            bool operator==(const Node3D &rhs) const;

            //Motion primitives
            // R = 6, 6.75 DEG
            std::vector<double> dy = {0., 0.0152, -0.0152}; 
            std::vector<double> dx = {0.174, 0.174, 0.174};
            std::vector<double> dyaw = {0, 0.174, -0.174};

        private:
            double x;
            double y;
            double yaw;
            double g; // cost so far
            double h; // cost to go
            const Node3D* pred;
            NodeType type = UNDEF;
            int idx;
            int primIdx; // index of the motion primitive used to generate this node
            
    };


}

#endif
