roslaunch simulation fake_robot.launch
rosrun simulation simple_tf_amcl.py
rosrun nav_staff amcl_adapted.py
rosrun nav_staff ReadMap.py
roslaunch machine 3D_RVIZ.launch

control:
roslaunch machine robot_controller_single.launch
rosrun simulation tele_handle_for_rviz.py


roslaunch simulation fake_robot.launch
roslaunch simulation fake_amcl.launch
roslaunch machine 3D_RVIZ.launch
roslaunch machine robot_controller_single.launch
rosrun simulation interactive_marker.py
rosrun nav_staff costplan_map.py
rosrun nav_staff Planner.py(改为直接订阅costplan map)
