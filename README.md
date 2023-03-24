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
```
$ roslaunch hybrid_a_star tune_hybrid_a_star.launch
```
Then you can set the initial and goal position in RViz.

# Reference
https://github.com/karlkurzer/path_planner

# TODO
- [ ] Add map inflation
- [ ] Add better heuristic