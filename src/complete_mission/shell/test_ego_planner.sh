gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch abot_bringup location.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch complete_mission octomap_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch complete_mission ego_planner_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch complete_mission ego_rviz.launch; exec bash"' \

