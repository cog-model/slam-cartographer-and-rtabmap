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
mkdir -p ~/cartographer_ws/src
cd ~/cartographer_ws/src
git clone http://github.com/andrey1908/cartographer
git clone http://github.com/andrey1908/cartographer_ros
git clone http://github.com/andrey1908/cartographer_example
git clone http://github.com/andrey1908/kas_utils
git clone http://github.com/andrey1908/slam_communication_msgs
```

* Build docker container:
```
cd ~/cartographer_ws/src/cartographer_ros/docker
./build.sh
```

* Build cartographer from docker:

Enter the container:
```
cd ~/cartographer_ws/src/cartographer_ros/docker
./start.sh
./into.sh
```
Build cartographer from container:
```
source /opt/ros/noetic/setup.bash 
cd ~/catkin_ws
catkin_make_isolated -DCMAKE_CXX_STANDARD=17 --use-ninja
```

---

**Running:**

Download [the map](https://disk.yandex.ru/d/uatW8OdyGZU6Yg) and place it to ```~/cartographer_ws``` folder:

Run cartographer:
```
cd ~/cartographer_ws/src/cartographer_example/scripts
python run_cartographer.py -l --load-map ~/cartographer_ws/5th_floor_with_polygon.pbstream
```

Input topics:
* /tf
* /tf_static
* /velodyne_points - lidar points

Output topics:
* /tf
* /cartographer/tracked_local_odometry - odometry
* /cartographer/tracked_global_odometry - pose on the map

Topics can be adjusted in ```~/cartographer_ws/src/cartographer_example/launch/cartographer.launch```.
