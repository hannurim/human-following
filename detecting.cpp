/**************************************************************
 * Writer : HAN NU RIM
 * Date : 2019.10.14 MON
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
    // leave 0~1.5 data/convert to (x,y)
    {
        int deg = (i + 180) % (total_deg + 1);
        if(msg->ranges[deg] && msg->ranges[deg] < 1.5) {
            s   = make_pair(-1 * msg->ranges[deg] * sin(deg*PI/180), msg->ranges[deg] * cos(deg*PI/180));
            raw_data.push_back(s);
        }
    }
    for(vector<int>::size_type i = 0; i < raw_data.size(); i++)
    // clustering data by distance(0.1)/leave data that get 0.05~0.2m width
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
            if (width < 0.2 && width > 0.05) {
                c1       = make_pair(s, f);
                clustered.push_back(pair<double, pair<pair<double, double>, pair<double, double> > >(width, c1));
            }
            start_flag   = true;
        }
    }
    for (vector<int>::size_type i = 0; i < clustered.size()-1; i++)
    // leave only data that may be human/distance between legs is 0.5m
    {
        s = make_pair((clustered[i].second.first.first + clustered[i].second.second.first) / 2,
                      (clustered[i].second.first.second + clustered[i].second.second.second) / 2);
        f = make_pair((clustered[i+1].second.first.first + clustered[i+1].second.second.first) / 2,
                      (clustered[i+1].second.first.second + clustered[i+1].second.second.second) / 2);
        dist = sqrt(pow(s.first - f.first,2) + pow(s.second - f.second,2));
        if (dist < 0.5) {
            p1 = make_pair(i,pair<double,double>((s.first+f.first)/2,(s.second+f.second)/2));
            total_p.push_back(p1);
        }
    }
    double  min     = 99999;
    int     index   = 0;
    for (vector<int>::size_type i = 0; i < total_p.size(); i++)
    // store the data of the nearest person
    {
        dist = sqrt(pow(total_p[i].second.first,2) + pow(total_p[i].second.second,2));
        if (min > dist) {
            min = dist;
            index = total_p[i].first;
            s = total_p[i].second;
        }
    }
    if (!total_p.empty()) {
        if (!p.goal.first || (abs(s.first - p.goal.first) < 0.5 && abs(s.second - p.goal.second) < 0.5))
        // update if it is similar to data stored previously(under 0.5m)
        {
            p.valid = true;
            p.goal = s;
            p.leg1.width = clustered[index].first;
            p.leg1.start = clustered[index].second.first;
            p.leg1.finish = clustered[index].second.second;
            p.leg2.width = clustered[index+1].first;
            p.leg2.start = clustered[index+1].second.first;
            p.leg2.finish = clustered[index+1].second.second;
        }
    }
    else {
        p.valid = false;
    }
    if (p.valid) {
        ROS_INFO("I DETECT HUMAN");
        GettingHuman();
    }
    else {
        ROS_ERROR("I LOST HUMAN");
        LostHuman();
    }
    cout << "p.goal.x = " << p.goal.first << "\t";
    cout << "p.goal.y = " << p.goal.second << endl;
    if (!raw_data.empty()) raw_data.clear();
    if (!clustered.empty()) clustered.clear();
    if (!total_p.empty()) total_p.clear();
}

void GettingHuman()
// if the human data is valid
{
    if (p.goal.first < -0.05)
        velOutput.angular.z = R_VEL;
    else if (p.goal.first > 0.05)
        velOutput.angular.z = -1 * R_VEL;
    if (p.goal.second > 0.3 && p.goal.second < 0.4)
    {
        ROS_INFO("STOP");
        velOutput.linear.x = 0;
    }
    else if (p.goal.second < 0.3) {
        ROS_INFO("BACKWARD");
        velOutput.linear.x = -1 * F_VEL;
    }
    else if (p.goal.second > 0.4) {
        ROS_INFO("FORWARD");
        velOutput.linear.x = F_VEL;
    }
}

void LostHuman()
// if the human data is not valid
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

    ros::Rate r(1000);

    while (ros::ok()) {
        pub.publish(velOutput);
        ros::spinOnce();
        r.sleep();
    }

    ros::spin();

    return 0;
}
