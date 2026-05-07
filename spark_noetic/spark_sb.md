
## 任务4
### 启动建图算法

```
roslaunch spark_rtabmap spark_rtabmap_teleop.launch
```

### 机器人上开rviz
```
rviz -d ~/Desktop/my_rviz_configs/my_robot.rviz
```

### 启动yolo
```
roslaunch yolo_ros yolo.launch
```

### 启动导航节点
```
roslaunch spark_navigation amcl_demo.launch map_file:=/home/spark/my_map.yaml
```

### 启动探索节点
```
cd /home/spark/exploration_ws/src/m-explore/explore/launch
roslaunch explore.launch
```
### 猫
```
rosrun door_finder cat_finder.py
```

## *Appendix*
 ### 键盘节点 
 0.25为线速度，0.5为角速度，用户可通过WASD控制Spark移动
```
rosrun spark_teleop spark_teleop_node 0.25 0.5 
```