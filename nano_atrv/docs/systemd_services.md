# Systemd Setup

To start the robot's ROS nodes when it boots, there's a service that runs the boot.launch.xml:

```bash
sudo systemctl status atrv_boot.service -n 5000
● atrv_boot.service
     Loaded: loaded (/etc/systemd/system/atrv_boot.service; enabled; preset: enabled)
     Active: active (running) since Thu 2026-03-26 14:04:34 CET; 6min ago
   Main PID: 15328 (atrv_boot.sh)
      Tasks: 190 (limit: 9064)
     Memory: 602.2M (peak: 685.5M)
        CPU: 11min 30.200s
     CGroup: /system.slice/atrv_boot.service
             ├─15328 /bin/bash /home/vicos/colcon_ws/src/nano_atrv/nano_atrv_bringup/systemd/atrv_boot.sh
             ├─15352 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch nano_atrv_bringup boot.launch.xml
             ├─15385 /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher 0 0 0.18 0 0 0 base_footprint base_link --ros-args -r __node:=tf_pub_1
             ├─15386 /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher -0.17 0 0 0 0 0 base_link rear_wheel --ros-args -r __node:=tf_pub_2
             ├─15387 /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher 0.17 0.115 -0.1 0 0 0 base_link left_wheel --ros-args -r __node:=tf_pub_3
             ├─15388 /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher 0.17 -0.115 -0.1 0 0 0 base_link right_wheel --ros-args -r __node:=tf_pub_4
             ├─15389 /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher 0 0 0.3 0 3.1415 0 base_link laser --ros-args -r __node:=tf_pub_4
             ├─15390 /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher 0.07 0 0.35 0 0 0 base_link camera_link --ros-args -r __node:=tf_pub_4
             ├─15392 /opt/ros/jazzy/lib/rclcpp_components/component_container --ros-args -r __node:=camera_container -r __ns:=/camera
             ├─15393 /home/vicos/colcon_ws/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node --ros-args -r __node:=ldlidar_stl_ros --params-file /tmp/launch_params__tvdu31b --params-file /tmp/launch_params_sjhte9_8 --params-file /tmp/launch_params_9n5oynyk --params-file /tmp/launch_params_3nwvy47u --params-file /tmp/launch_params_z55fy30j --params-file /tmp/launch_params_ml6wcrup --params-file /tmp/launc>
             ├─15394 python3 /opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket --ros-args -r __node:=vizanti_rosbridge --params-file /tmp/launch_params_e8pewzsu --params-file /tmp/launch_params_yasg95wx --params-file /tmp/launch_params_6r4gkdsr --params-file /tmp/launch_params_7n3azbpt --params-file /tmp/launch_params_xal_58ub --params-file /tmp/launch_params_4_sg7l29 --params-file /tmp/launch_params_lqtg26xh --para>
             ├─15399 python3 /opt/ros/jazzy/lib/rosapi/rosapi_node --ros-args -r __node:=rosapi
             ├─15400 python3 /home/vicos/colcon_ws/install/vizanti_server/lib/vizanti_server/server.py --ros-args -r __node:=vizanti_flask_node --params-file /tmp/launch_params_yffuxqr0 --params-file /tmp/launch_params_k4qgk5zc --params-file /tmp/launch_params_90e05eu1 --params-file /tmp/launch_params_p74x7v3b --params-file /tmp/launch_params_42ykfdyc --params-file /tmp/launch_params_3bcyom25 --params-file /tmp/launch_par>
             ├─15401 /home/vicos/colcon_ws/install/vizanti_cpp/lib/vizanti_cpp/tf_consolidator --ros-args -r __node:=vizanti_tf_handler_node
             └─15406 python3 /home/vicos/colcon_ws/install/vizanti_server/lib/vizanti_server/service_handler.py --ros-args -r __node:=vizanti_service_handler_node

Mar 26 14:04:34 mrcrabs systemd[1]: Started atrv_boot.service.
Mar 26 14:04:35 mrcrabs atrv_boot.sh[15352]: [INFO] [launch]: All log files can be found below /home/vicos/.ros/log/2026-03-26-14-04-35-621765-mrcrabs-15352
Mar 26 14:04:35 mrcrabs atrv_boot.sh[15352]: [INFO] [launch]: Default logging verbosity is set to INFO
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [static_transform_publisher-1]: process started with pid [15385]
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [static_transform_publisher-2]: process started with pid [15386]
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [static_transform_publisher-3]: process started with pid [15387]
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [static_transform_publisher-4]: process started with pid [15388]
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [static_transform_publisher-5]: process started with pid [15389]
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [static_transform_publisher-6]: process started with pid [15390]
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [feetech_servo_node.py-7]: process started with pid [15391]
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [component_container-8]: process started with pid [15392]
Mar 26 14:04:36 mrcrabs atrv_boot.sh[15352]: [INFO] [ldlidar_stl_ros2_node-9]: process started with pid [15393]
```

## Some example patterns:

Restart the main launch file to apply any changes to boot.launch.xml or ros_env.sh:

```bash
sudo systemctl restart atrv_boot.service
```

Disable service temporarily so the launch can be ran manually:

```bash
sudo systemctl stop atrv_boot.service
sudo systemctl disable atrv_boot.service
```

Enable service and start:

```bash
sudo systemctl enable atrv_boot.service
sudo systemctl start atrv_boot.service
```