# Testing
The package contains tests for the toposens_task_node which is the parser. 

## Running the tests
```console
mkdir -p ~/catkin_ws/src
cd src
git clone https://github.com/ashnarayan13/toposens_reader.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -s  # do a dry-run first
rosdep install --from-paths src --ignore-src -r -y
catkin_make 
catkin_make run_test_toposens_task
```
