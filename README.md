# denso_robot_ros

To find current cartesian coordinate of EE :

rosrun tf tf_echo <your_base_link> <your_gripper_frame>

or in the VS050's case, run

rosrun tf tf_echo base_link J6

To start up controlling nodes for VS050:

roslaunch denso_robot_bringup vs050_bringup.launch sim:=true ip_address:=192.168.0.1

To run 3D printing module (NOTE: the extrusion by ROS is still not solved):

roslaunch denso_robot_3dp denso_robot_3dp.launch gcode_file:=resources/single_layer_square.gcode

