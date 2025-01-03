# Hybrid A Star Algorithm
This is a ROS-independent sub project of [PathPlanner](https://github.com/karlkurzer/path_planner.git)

## Prerequisite
* [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/)

## Setup
```
sudo apt install libompl-dev
```

## Run
```
mkdir build && cd build
cmake .. && make -j
./example_hybrid_astar_algorithm
```