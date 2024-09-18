Inception - Project 0
Abhinav Bhamidipati 


Steps to run the package

1. colcon build
2. source install/setup.bash

3. ros2 launch inception_p0 gazebo_multi_nav2.launch.py  - #launches the world with robots and also launches NAV2 

4. Open a new terminal 
    ros2 launch inception_p0 global.launch.py - #launches the red cube detection, yolo human detection and updates the detected red cubes as obstacles for all robots

5. New terminal
    ros2 launch inception_p0 autonomy.launch.py - #launch the behavior tree
    
    

Google Drive Link : https://drive.google.com/drive/folders/137Py9VkAu7WxhED87a7wK2qzWCGzrjDn?usp=sharing 

	
	
On launching all the launch files, the robots will start moving around the map to the given locations in locations.yaml file. The robots will add the red cubes as a obstacles in the costmap which will update for both of the robots. The pink circles depict the red cubes added as obstacles. While navigating around the map if the robot detects a human using the YOLO model, the robots will stop navigating and go towards the person. 


The behavior tree is defined as following:
	- Parallel Node runs two tasks simultaneously
		1. 'CheckForPerson' detects if a person is present 
		2. 'navigate_sequence': navigates sequentially to predefined locations in the .yaml file.
	- 'GoToPose (navigate_to_person): If a person is detected, the robot will navigate to the person's location.
	
	

- Change the spawn location of the robot in the gazebo_multi_nav2.launch.py. 
- We can change the radius of the hazardous zone in the red_cube_detect.py.
 





