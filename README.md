RPLIDAR ROS package
=====================================================================

ROS node and test application for RPLIDAR

Visit following Website for more details about RPLIDAR:

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage:   http://www.slamtec.com/en/Lidar

rplidar Tutorial:  https://github.com/robopeak/rplidar_ros/wiki



    (1) 打开终端执行：
        source /opt/ros/foxy/setup.bash
    (2) 进入/root/app/rplidar_ros2/out，执行：
        colcon build --merge-install --base-path .. --cmake-args -DCOMMON_CMAKE=/root/app/include/cscv/cmaker/cscv.cmake
    (3) 打开新的终端启动launch文件：
    source /root/app/ros2ex/rplidar_ros2/out/install/setup.bash
    ros2 launch rplidar_ros2 rplidar.launch.py
