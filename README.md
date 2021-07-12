# toposens_reader
This repository contains 2 libraries, **topo_parser** - to read the data file and publish the point cloud.
The second node, **topo_cluster** is to read the point cloud published by topo_parser and publishes a boolean output if the pole is found.
## Dependencies
- [pcl_ros](http://wiki.ros.org/pcl_ros) 
- [pcl_conversions](http://wiki.ros.org/pcl_conversions) 
## How to run it
```console
mkdir -p ~/catkin_ws/src
cd src
git clone https://github.com/ashnarayan13/toposens_reader.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -s  # do a dry-run first
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
roslaunch toposens_task toposens.launch
```

## Tuning
The **topo_parser** requires a file name for reading the data file. This can be set from the ros parameter interface from the launch.

The **topo_cluster** requires 3 parameters. The cluster tolerance, min cluster size and max cluster size. This can also be set with the ros parameter interface in the launch file.

## Output
The toposens_task_cluster_node publishes a bool message on /topo/poleStatus 

The toposens_task_node publishes the pointcloud on the topic /topo/pointCloud