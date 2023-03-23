#include <iostream>
#include <vector>
#include <memory>

class Node3D
{
    public: 
        Node3D(double cost, int id): cost_(cost), id_(id) {}
        double getC() const { return cost_; }
        int getID() const { return id_; }
    private:
        double cost_;
        int id_;
};

struct CompareNodes
{
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const Node3D *lhs, const Node3D *rhs) const
    {
        return lhs->getC() > rhs->getC();
    }
};


int main()
{

    std::vector<int> v(10);
    for (int i = 0; i < 10; ++i)
    {
        std::cout << v[i] << std::endl;
    }

}