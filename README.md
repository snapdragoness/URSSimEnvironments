Reason for choosing hector_quadrotor and not tum_simulator for Ardrone :  
http://answers.ros.org/question/62981/multiple-ardrones-on-tum_simulator/

Installation guide and tips will be written soon...

For now, assume that you setup everything correctly (ros, gazebo_ros, bashrc), you can try spawning drones by  
$ roslaunch hector_quadrotor_gazebo spawn_four_quadrotors.launch

Sometimes when you terminate the programs (Gazebo, rviz, etc), the nodes do not get cleared from master node's record, you can still see them by  
$ rosnode list  
and if you want to clean them out, use  
$ rosnode cleanup
