# GPS-pure-puirsuit 
ROS package to simulate GPS waypoints navigation and pure puirsuit path following

---

## How to build ##

Clone the repo in catkin_ws/src and `catkin_make`

## How to run ##

The use of this package you have to install `move_base` framework
for navigation. To run the vehicle simulation 

```
roslaunch mathew test_pure_puirsuit
```

To run the waypoint navigation node

```
roslaunch mathew gps_waypoint_node
```



