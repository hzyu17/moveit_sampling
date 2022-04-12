USE MOVEIT TO COLLECT ROBOT TRAJECTORY DATA
--------------------------------------------

**Collect trajectory data from a 7-DOF pandas robot arm in a Rviz environment with cluttered obstacles. Motion planning pipeline is OMPL-CHOMP integrated in moveit.**

 * Tutorial for Moveit: 
[Moveit step by step tutorial](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
 . This project has been tested on Ubuntu18.04 ROS melodic with moveit1.
 * Requirements
Ubuntu18.04
[ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
 * Usage: After creating the neccessary ROS environments and installation of moveit
   * run the rviz world: 
 	<p><code> roslaunch panda_moveit_config demo.launch pipeline:=ompl-chomp </code></p>
   * adding obstacles:
<html>
 <head>
	python ~/ws_moveit/src/moveit_collect_data/scripts/collision_scene.py
 </head>
</html>
   * start the data listener:
<html>
 <head>
	python ~/ws_moveit/src/moveit_collect_data/scripts/collect_data.py
 </head>
</html>
   * start sampling node:
<html>
 <head>
	rosrun moveit_collect_data random_sampling
 </head>
</html>

