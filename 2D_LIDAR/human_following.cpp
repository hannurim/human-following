/**************************************************************
 * Writer : HAN NU RIM
 * Date : 2019.11.20 WED
 * Input : Scan data(Lidar)
 * Output : If (the width of each legs is between 5 - 20cm) and
 *             (the distance of legs is under 50cm),
 *          Turtlebot3 will recognize that data as human and follow it.
 * ************************************************************/
#include "detecting.h"

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int     total_deg   = msg->angle_max / msg->angle_increment;
    for (int i = 0; i < total_deg+1; i++)
    // Coordination
    {
        int deg = (i + 180) % (total_deg + 1);
        if(msg->ranges[deg] && msg->ranges[deg] < 1.5){ // Leave 0~1.5 data
            s   = make_pair(-1 * msg->ranges[deg] * sin(deg*PI/180), msg->ranges[deg] * cos(deg*PI/180));
            raw_data.push_back(s);
        }
    }
    for(vector<int>::size_type i = 0; i < raw_data.size(); i++)
    // Clustering
    {
        dist = sqrt(pow(raw_data[i].first-raw_data[i+1].first, 2) +
                    pow(raw_data[i].second-raw_data[i+1].second, 2));
        if (start_flag && dist < 0.1) {
            s           = make_pair(raw_data[i].first,raw_data[i].second);
            start_flag  = false;
        }
        else if (!start_flag && dist > 0.1) {
            f = make_pair(raw_data[i].first,raw_data[i].second);
            double width = sqrt(pow(s.first-f.first,2) + pow(s.second-f.second,2));
            if (width < 0.2 && width > 0.05) { // Save data that get 0.05~0.2m width
                c1       = make_pair(s, f);
                clustered.push_back(pair<double, pair<pair<double, double>, pair<double, double> > >(width, c1));
            }
            start_flag   = true;
        }
    }
    for (vector<int>::size_type i = 0; i < clustered.size()-1; i++)
    // Extract person candidate
    {
        if (clustered.size() == 0) break;
        s = make_pair((clustered[i].second.first.first + clustered[i].second.second.first) / 2,
                      (clustered[i].second.first.second + clustered[i].second.second.second) / 2);
        f = make_pair((clustered[i+1].second.first.first + clustered[i+1].second.second.first) / 2,
                      (clustered[i+1].second.first.second + clustered[i+1].second.second.second) / 2);
        dist = sqrt(pow(s.first - f.first,2) + pow(s.second - f.second,2));
        if (dist < 0.5) { // If distance between legs is 0.5m, May be the person
            p1 = make_pair(i,pair<double,double>((s.first+f.first)/2,(s.second+f.second)/2));
            total_p.push_back(p1);
        }
    }
    if (!total_p.empty()) {
        if (p.goal.first) Association();            // If the data have a previous data
        else if (!p.goal.first) Identification();   // If the data is initialized
    }
    else err_cnt++;
    if (p.valid) {
        ROS_INFO("I DETECT HUMAN");
        GettingHuman();
    }
    else {
        ROS_ERROR("I LOST HUMAN");
        LostHuman();
    }
    if (err_cnt > 50) p.goal.first = 0;     // If the robot doen't find a person for 10 seconds, It initializes
    cout << "p.goal.x = " << p.goal.first << "\t";
    cout << "p.goal.y = " << p.goal.second << endl;
    if (!raw_data.empty()) raw_data.clear();
    if (!clustered.empty()) clustered.clear();
    if (!total_p.empty()) total_p.clear();
    if (!p_list.empty()) p_list.clear();
}

void Association()
// Update to the nearest value within 1.0m of existing value
{
    for (vector<int>::size_type i = 0; i < total_p.size(); i++) {
        dist = sqrt(pow(total_p[i].second.first - p.goal.first, 2) +
                    pow(total_p[i].second.second - p.goal.second, 2));
        if (dist < 1.0) // Determine If it's within 1.0m radius
            p_list.push_back(total_p[i].second);
    }
    if (!p_list.empty()) {
        int index = GetMin(p_list);
        p.goal.first = p.goal.first * Kl + p_list[index].first * (1 - Kl);      // Smoothing filter
        p.goal.second = p.goal.second * Kl + p_list[index].second * (1 - Kl);
        p.valid = true;
        err_cnt = 0;
    }
}

void Identification()
// Update data only when a person is detected near 0.4m ahead
{
    int index = GetMin(total_p);
    if (total_p[index].second.second > 0.4) p_list.push_back(total_p[index].second);
    if (!p_list.empty()) {
        p.goal.first = p.goal.first * Kl + p_list[0].first * (1 - Kl);      // Smoothing filter
        p.goal.second = p.goal.second * Kl + p_list[0].second * (1 - Kl);
        p.valid = true;
        err_cnt = 0;    // Reset error count
    }
    else p.valid = false;
}

void GettingHuman()
// When a person is detected
{
    double theta_target = atan2(p.goal.second, p.goal.first);
    double theta_error = theta_target - R_ANG;
    double dist_target = p.goal.second;
    double dist_error = dist_target - R_DIS;
    velOutput.angular.z = Kw * theta_error;
    velOutput.linear.x = Kv * dist_error;
}

void LostHuman()
// When a person is not detected
{
    velOutput.linear.x = 0;
    velOutput.angular.z = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detecting");
    ros::NodeHandle nh;

    ROS_INFO("detecting start");

    ros::Subscriber sub = nh.subscribe("scan", 10, sensorCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate r(5);

    while (ros::ok()) {
        pub.publish(velOutput);
        ros::spinOnce();
        r.sleep();
    }

    ros::spin();

    return 0;
}
