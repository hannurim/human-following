# Human Following using 2D Lidar
Operates turtlebot3 in ROS

I used 2D Lidar.

First, The vector of ***raw_data*** have valid data. It was not zero, it was less than 1.5m. (Radius, Theta) converted the data into (x, y).

Second, The vector of ***clustered*** have data likely to be a leg. The standard for clustering is more than 10cm apart. Values are stored when the width is more than 5cm and less than 20cm.

Third, The vector of ***total_p*** have data likely to be person. If the distance of legs are under 50cm, The data of goal is saved.

    GOAL = (LEG1.CENTER + LEG2.CENTER) / 2
    
Fourth, The class of ***p*** has one of the closest ***total_p*** data values. Update occurs when the previous value are not different up to 50cm.

You can control forward velocity(F_VEL) and rotate velocity(R_VEL).
All you have to do is roslaunch turtlebot3_bringup and rosrun this code.

Remove Segmentation Fault(2019.10.18) :
Segmentation fault is occurred when 2D Lidar have no data.
So, Clear the vectors only when the vectors is not empty.

Add Several Algorithm(2019.11.20) : 
1. Use Smoothing filter for smooth driving.
2. Do not drive until a person is detected 0.4m ahead.
3. New tracking algorithm : The list of ***p_list*** has value within 0.3m around the existing value.
Update to the nearest value in the list.
4. Method to reduce noise : Real new goal = Previous goal * Weight + New goal * (1 - Weight)
5. Initialize the value if no person is found for 10 seconds
