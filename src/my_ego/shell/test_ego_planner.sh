gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch my_ego location.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch my_ego octomap_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch my_ego ego_planner_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch my_ego ego_rviz.launch; exec bash"' \

