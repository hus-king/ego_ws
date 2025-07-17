gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch ego_planner location.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch ego_planner octomap_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch ego_planner ego_planner_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch ego_planner ego_rviz.launch; exec bash"' \

