MAKE SURE YOU CHANGE FILE PATHS TO YOUR COMPUTER IN FILES!!!

# Turtlebot3_MultiAgent_ROS
Creating a dashboard to control several robotic platforms through ROS, simplifying multi-agent machine learning

For this project we used ROS Noetic with Rviz and Turtlebot3


Changes:
Wanted to modify robot files to enable multi robot navigation. So main files that needed editing were the files in navigation: move_base, turtlebot3_navigation, and amcl

Changed files:
single_gmapping_turtlebot3.launch: Created from multi_turtlebot3.launch. Essentially just keeps first_tb3 robot and gets rid of other robot definitions.
turtlebot3_navigation.launch: AMCL launch includes scan_topic argument that specifies the namespace for the /scan topic.
Move_base.launch: added params to reset frame_id parameters using user input data. Sets costmaps, planners, goal, move_base topics to be robot namespace specific, so multiple robots have distinct move_base and can be controlled individually. 
Added group namespace for other params so the namespace specific topics are controlled.
Multi_map_merge.launch: New file created that can be run however many times as required for the number of robots launched, as original multi_map_merge.launch hard coded 3 maps to be merged. Added static transform to merge individual robot specific maps to the general map topic. Parameters for the  map’s coordinates were shifted a little in the transform to fix the misalignment issue.
Amcl.launch: has position added so it already knows robot locations at spawn.
MoveCoordinate: In Main where it moves a specified number of robots to a certain location.
Move_bot: In the Demo_laptop branch where it moves one specified robot to a specific location given (robot_name,x,y)

Each py file will have comments explaining the changes:

That is the main file for navigation, the other files have similar functions that act the same.

The world that is launched is the one for “turtlebot3_pentagon” which has a pentagon ring with obstacles in the middle. We took out the bot that already spawns so we are able to spawn our own but they are pretty similar.
In launching the robots, the file was changed to spawn one at a time and give each one a robot state publisher and the prefix of its name so it's not confused with other bots. The spawn location is given in the arguments.
Turtlebot3_navigation was changed to not spawn a map originally in order to have the proper setup for the gmapping and finding new locations with the robots in the environment. AMCL has the initial position added so it already knows where the robots were added. Move_base was next and the name of the robot was added. More will be known when talking about the move_base file in 4. Rviz had the file changed to auto bring up tb3_0 with movement and the scan and everything else associated.
Move_base was the most changed out of all the files. In order to have multiple robots being able to use move_base on their own /cmd_vel topics and scanner topics, we created a namescape for the file which changed most of the topic names to run only on its own robot. At the bottom we changed all of the values to match the robot in particular and make it all a part of the map so it is linked. Then we remapped move_base so we can change it in Rviz and the parameters are linked to the move_base package for each individual robot.
The map merge file that is run multiple times also has changes. It was cut down to 1 robot with its initial position as the arguments. We added a static_transform_publisher for the transform to get the maps from each robot to link to the overall map. We noticed that the map was slightly off and changing the robot’s starting position didn’t change it so we changed the location of the transform to match the one in the simulation. This will probably have to be changed in the future but there was no other fix found for this problem.

