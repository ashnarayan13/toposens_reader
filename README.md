# toposens_reader
This repository contains 2 nodes, **toposens_parse** - to read the data file and publish the point cloud.
The second node, **toposens_cluster** is to read the point cloud published by toposens_parse and publishes a boolean output if the flag is found.
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

