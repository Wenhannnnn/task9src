## Useful packages
gz_example_robot_description
leo_common-ros2/leo_description
navigation_demos
## launch
```python
ros2 launch navigation_demos nav_demo.launch.py
```
## file relationships
navigation_demos nav_demo.launch.py will launch sim_robot.launch.py in package gz_example_robot_description
sim_robot.launch.py will spawn the robot model from gz_example_robot_description/urdf/leo_sim.urdf.xacro
gz_example_robot_description/urdf/leo_sim.urdf.xacro use another urdf file, leo_common-ros2/leo_description/urdf/macros.xacro
this procedure above will be simplified later..
