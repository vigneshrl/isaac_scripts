# command to run pointcloud_to_laserscan
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node     --ros-args     -p target_frame:=sim_lidar     -p min_height:=-1.0     -p max_height:=1.5     -p range_min:=0.1     -p range_max:=30.0     -p angle_min:=-3.1416     -p angle_max:=3.1416     -p scan_time:=0.05     -p use_sim_time:=true     -r cloud_in:=/os_cloud_node/points     -r scan:=/scan
