1st run launch file after buildingL:- ros2 launch final_project final_project.launch.py


then run the part_publisher node arom another terminal:- ros2 run part_publisher part_publisher_exe

in third terminal, run for transformation & navigation nodes:-    ros2 run task2 part_subscriber_exe


Note: All part's world frame pose to be visited is found and stored in Parts vector defined in camera_broadcaster
For some reason, it was unable to be published to turtlebot for navgation. So, turtlbot is moving towards 1st part only. It is stoppng after that
