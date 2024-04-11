**Installation:**

* Install dependencies:
```
# subprocess-tee
cd ~
git clone http://github.com/andrey1908/subprocess-tee
cd ~/subprocess-tee
pip install .

# docker_helper
cd ~
git clone http://github.com/andrey1908/docker_helper
cd ~/docker_helper
pip install .
```

* Clone repositories:
```
mkdir -p ~/rtabmap_ws/src
cd ~/rtabmap_ws/src
git clone http://github.com/andrey1908/rtabmap
git clone http://github.com/andrey1908/rtabmap_ros
git clone http://github.com/andrey1908/rtabmap_example
git clone http://github.com/andrey1908/kas_utils
git clone http://github.com/andrey1908/slam_communication_msgs
git clone http://github.com/andrey1908/colored_occupancy_grid
```

* Build docker container:
```
cd ~/rtabmap_ws/src/rtabmap_ros/docker
./build.sh
```

* Build rtabmap from docker:

Enter the container:
```
cd ~/rtabmap_ws/src/rtabmap_ros/docker
./start.sh
./into.sh
```
Build rtabmap from container:
```
source /opt/ros/noetic/setup.bash 
cd ~/catkin_ws
catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

---

**Running:**

Download [the map](https://disk.yandex.ru/d/_1znahOXJN00rw) and place it to ```~/rtabmap_ws``` folder.

Run rtabmap:
```
cd ~/rtabmap_ws/src/rtabmap_example/scripts
python run_rtabmap.py --load-map ~/rtabmap_ws/5th_floor_with_polygon.ocp
```

Input topics:
* /tf
* /tf_static
* /velodyne_points - lidar points
* /cartographer/tracked_local_odometry - odometry
* /cartographer/tracked_global_odometry - pose on the map

Output topics:
* /occupancy_grid_map/grid_map - occupancy map

Topics can be adjusted in ```~/rtabmap_ws/src/rtabmap_example/launch/occupancy_grid_map.launch```.
