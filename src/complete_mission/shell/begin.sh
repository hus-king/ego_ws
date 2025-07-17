tmux new-session -d -s begin
tmux send-keys -t begin 'echop' C-m
tmux send-keys -t begin 'exec bash' C-m

# 创建第一个水平分割窗格
tmux split-window -v -t begin
tmux send-keys -t begin.1 'roslaunch complete_mission complete_mission.launch' 

tmux setw pane-base-index 0
tmux select-layout even-vertical 

tmux attach-session -t begin