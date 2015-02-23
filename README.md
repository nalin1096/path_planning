# path_planning
A path planning algorithm based on RRT implemented using ROS.

distro - indigo <br  />

The algorithm find an optimized path for a one obstacle environment. The visualtization is done in RVIZ and the code is written in C++.  <br  />

The package has two executables: <br  />
1. ros_node <br  />
2. env_node <br  /><br  />

RVIZ parameters:  <br  />
1. Frame_id = "/path_planner"  <br  />
2. marker_topic = "path_planner_rrt"  <br  />

Instructions:  <br  />

1. Open terminal and type  <br  />
  $roscore  <br  />
2. Open new terminal and go to the the root of your catkin workspace  <br  />
  $catkin_make  <br  />
  $source ./devel/setup.bash  <br  />
  $rosrun path_planner env_node  <br  />
3. open new terminal  <br  />
  $rosrun rviz rviz  <br  />
4. In the RVIZ window, change:  <br  />
  fixed frame under global option to "/path_planner"  <br  />
  add a marker and change marker topic to "path_planner_rrt"  <br  />
5. Open new terminal  <br  />
  $rosrun path_planner rrt_node  <br  />

The environment and the path will be visualized in RVIZ.   <br  />
