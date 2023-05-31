# Hybrid A* Planner
A reproduction of Hybrid A* algorithm for autonomous vehicles.

# Installation
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/SailorBrandon/hybrid_a_star.git
$ cd .. && catkin_make
$ source devel/setup.bash 
```

# How to use
This planner is written as a [global planner plugin](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS) of the ROS Navigation Stack. To use it, simply add the following tag to your launch file:
```
<node pkg="move_base" type="move_base" name="move_base" output="screen">
  <param name="base_global_planner" value="hybrid_a_star/HybridAStar" />
  ...
</node>
```

# Reference
- How to design the planner: https://github.com/karlkurzer/path_planner
- How to write the planner as a ROS plugin: https://github.com/dengpw/hybrid_astar_planner

# TODO
- [ ] Add map inflation
- [ ] Add better heuristic
