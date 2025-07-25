#!/bin/bash

# EGO Planner 启动脚本
# 只保留必要的核心组件

# 创建会话和主要节点窗口
tmux new-session -d -s ros_session -n main_nodes

# Pane 0: ROS核心
tmux send-keys -t ros_session:0 'roscore' C-m

# Pane 1: 定位系统
tmux split-window -h -t ros_session:0
tmux send-keys -t ros_session:0.1 'sleep 3; roslaunch abot_bringup location.launch' C-m

# Pane 2: 深度相机
tmux split-window -v -t ros_session:0.1
tmux send-keys -t ros_session:0.2 'sleep 4; roslaunch astra_camera gemini.launch' C-m

# Pane 3: EGO规划器
tmux split-window -v -t ros_session:0.0
tmux send-keys -t ros_session:0.3 'sleep 10; roslaunch complete_mission ego_planner_mid360.launch' C-m

# Pane 4: 激光雷达坐标转换
tmux split-window -h -t ros_session:0.3
tmux send-keys -t ros_session:0.4 'sleep 12; rosrun complete_mission laser_to_worldframe' C-m

# 整理主窗口布局
tmux select-layout -t ros_session:0 tiled

# --------------------
# 监控窗口
# --------------------
tmux new-window -t ros_session:1 -n monitors

# Pane 0: 位置监控
tmux send-keys -t ros_session:1 'sleep 4; rostopic echo /mavros/local_position/pose' C-m

# Pane 1: EGO规划器任务启动
tmux split-window -v -t ros_session:1
tmux send-keys -t ros_session:1.1 'sleep 7; roslaunch my_ego ego_planner.launch' C-m

# 整理监控窗口布局
tmux select-layout -t ros_session:1 tiled

# 附加到会话并显示主窗口
tmux select-window -t ros_session:0
tmux attach-session -t ros_session
