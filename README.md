# Human Following
Operates turtlebot3 in ROS

I implemented the same performance in two ways.
The first uses a 2D Lidar and the next uses a depth information in realsense D435.

First, the valid value is deduced.
Secondly, I only identify objects that can be human leg.
Thirdly, if the distance between the legs is less than 50cm, it is recognized as one person.
Finally, save the nearest person as the target point.
Considering the stride length of a person, the value is updated only when the target point is less than 50cm from the previous value.

The 2D Lidar will follow a person's target at a distance of 30-40cm.

The driving part of realsense is incomplete.
I used only one line of information in realsense.
Because I wanted to behave like the 2D Lidar.
