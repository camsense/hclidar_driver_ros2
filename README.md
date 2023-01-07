![hclidar](images/hclidar.jpg  "hclidar")
# hclidar ROS2 Driver

hclidar_driver_ros2 is a new ros package, which is designed to gradually become the standard driver package for hclidar devices in the ros2 environment.



## Clone hclidar_driver_ros2

1. Clone hclidar_driver_ros2 package for github : 

   `git clone https://github.com/camsense/hclidar_driver_ros2.git hclidar_ros2_ws/src/hclidar_driver_ros2`

2. Build hclidar_driver_ros2 package :

   ```
   cd hclidar_ros2_ws
   colcon build --symlink-install
   ```
   Note: install colcon [see](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon)



3. Package environment setup :

   `source ./install/setup.bash`

    Note: Add permanent workspace environment variables.
    It's convenientif the ROS2 environment variables are automatically added to your bash session every time a new shell is launched:
    ```
    $echo "source ~/hclidar_ros2_ws/install/setup.bash" >> ~/.bashrc
    $source ~/.bashrc
    ```
4. Confirmation
    To confirm that your package path has been set, printenv the `grep -i ROS` variable.
    ```
    $ printenv | grep -i ROS
    ```
    You should see something similar to:
        `OLDPWD=/home/konyun/hclidar_ros2_ws/install`


	
## Configure LiDAR [paramters](params/hclidar.yaml)
```
hclidar_driver_ros2_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 230400
    lidar_type: 1
    device_type: 0
    sample_rate: 9
    abnormal_check_count: 4
    resolution_fixed: true
    reversion: true
    inverted: true
    auto_reconnect: true
    isSingleChannel: false
    intensity: false
    support_motor_dtr: false
    angle_max: 3.1415926
    angle_min: -3.1415926
    range_max: 64.0
    range_min: 0.01
    frequency: 10.0
    invalid_range_is_inf: false
```

## Run hclidar_driver_ros2

##### Run hclidar_driver_ros2 using launch file

The command format is : 

 `ros2 launch hclidar_driver_ros2 [launch file].py`

1. Connect LiDAR uint(s).
   ```
   ros2 launch hclidar_driver_ros2 hclidar_launch.py 
   ```
   or 

   ```
   launch $(ros2 pkg prefix hclidar_driver_ros2)/share/hclidar_driver_ros2/launch/hclidar.py 
   ```
2. RVIZ 
   ```
   ros2 launch hclidar_driver_ros2 hclidar_launch_rviz.py 
   ```


3. echo scan topic
   ```
  ros2 topic echo /scan
   ```

#####  Launch file introduction

The driver offers users a wealth of options when using different launch file. The launch file directory    

is `"hclidar_ros2_ws/src/hclidar_driver_ros2/launch"`. All launch files are listed as below : 

| launch file               | features                                                     |
| ------------------------- | ------------------------------------------------------------ |
| hclidar.py         | Connect to defualt paramters<br/>Publish LaserScan message on `scan` topic |
| hclidar_launch.py         | Connect hclidar.yaml Lidar specified by configuration parameters<br/>Publish LaserScan message on `scan` topic |
| hclidar_launch_rviz.py         | Connect hclidar.yaml Lidar specified by configuration parameters and setup RVIZ<br/>Publish LaserScan message on `scan` topic |



## Publish Topic
| Topic                | Type                    | Description                                      |
|----------------------|-------------------------|--------------------------------------------------|
| `scan`               | sensor_msgs/LaserScan   | 2D laser scan of the 0-angle ring                |



## Configure hclidar_ros_driver internal parameter

The hclidar_driver_ros2 internal parameters are in the launch file, they are listed as below :


## Contact Camsense


If you have any extra questions, please feel free to [contact us](http://www.camsense.cn)






