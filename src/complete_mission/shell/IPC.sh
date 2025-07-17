gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch simulation iris_mid360_house.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch fast_lio mapping_mid360.launch; exec bash"' \
# --tab -e 'bash -c "sleep 4; roslaunch complete_mission ipc.launch; exec bash"' \
# --tab -e 'bash -c "sleep 5; rosrun complete_mission IPC_test; exec bash"' 
# --tab -e 'bash -c "sleep 4; roslaunch complete_mission octomap_mid360.launch; exec bash"' \
