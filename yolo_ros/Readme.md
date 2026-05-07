# 启动每个节点的命令如下
## 机器人spark端
见spark的包里
## laptop 端
### 启动yolo
```
roslaunch yolo_ros yolo.launch
```

### 启动探索节点

```
roslaunch explore_lite explore.launch
```

### yolo标图
```
roslaunch yolo_rviz_markers yolo_to_rviz.launch
```

### 找猫
```
roslaunch find_cat find_cat.launch
```

## *Appendix*
 ### 键盘节点 
 0.25为线速度，0.5为角速度，用户可通过WASD控制Spark移动
```
rosrun spark_teleop spark_teleop_node 0.25 0.5 
```
