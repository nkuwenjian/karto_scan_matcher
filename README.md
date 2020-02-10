# karto_scan_matcher
A C++ library that wraps Karto's laser scan matcher without loop closure.

## How to use on Ubuntu?
    1. This package has been tested well on Ubuntu 16.04 LTS, and the version of ROS is kinetic.
        
    2. Clone and catkin_make
        $ cd ~/catkin_ws/src/
        $ git clone https://github.com/nkuwenjian/karto_scan_matcher.git
        $ cd ..
        $ catkin_make -j4

    3. Run offline rosbag
        $ roslaunch karto_scan_matcher demo.launch
        $ rosbag play <rosbagfile> --clock
        
## Topics

### Subscribed topics
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html))

### Published topics
- `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))
- `/map_metadata` ([nav_msgs/MapMetaData](http://docs.ros.org/melodic/api/nav_msgs/html/msg/MapMetaData.html))
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html))
        
## Thanks

[1] https://github.com/ros-perception/open_karto

[2] https://github.com/ros-perception/slam_karto
