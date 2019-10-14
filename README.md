# Human Following
Operates turtlebot3 in ROS

I used 2D Lidar.

First, The vector of ***raw_data*** have valid data. It was not zero, it was less than 1.5m. (Radius, Theta) converted the data into (x, y).

Second, The vector of ***clustered*** have data likely to be a leg. The standard for clustering is more than 10cm apart. Values are stored when the width is more than 5cm and less than 20cm.

Third, The vector of ***total_p*** have data likely to be person. If the distance of legs are under 50cm, The data of goal is saved.

    GOAL = (LEG1.CENTER + LEG2.CENTER) / 2
    
Fourth, The class of ***p*** has one of the closest totap_p data values. Update occurs when the previous value are not different up to 50cm.

You can control forward velocity(F_VEL) and rotate velocity(R_VEL).
All you have to do is roslaunch turtlebot3_bringup and rosrun this code.
