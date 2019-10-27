#include "detecting_depth.h"

void colorCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    for (int i = 0; i < msg->width; i++)
    // leave between 0cm and 150cm data/convert to (x,y)
    {
        float deg = (i - 319) * 45 / 320;
        dist = (msg->data[LINE*STEP+2*i] + msg->data[LINE*STEP+2*i+1] * 256) * 0.1;
        if(dist > 0 && dist < 150) {
            s   = make_pair(dist * sin(deg*PI/180), dist * cos(deg*PI/180));
            raw_data.push_back(s);
        }
    }
    for(vector<int>::size_type i = 0; i < raw_data.size()-1; i++)
    // clustering data by distance(10cm)/leave data that get 5~20cm width
    {
        dist = sqrt(pow(raw_data[i].first-raw_data[i+1].first, 2) +
                    pow(raw_data[i].second-raw_data[i+1].second, 2));
        if (start_flag && dist < 5) {
            s           = make_pair(raw_data[i].first,raw_data[i].second);
            start_flag  = false;
        }
        else if (!start_flag && dist > 5) {
            f = make_pair(raw_data[i].first,raw_data[i].second);
            double width = sqrt(pow(s.first-f.first,2) + pow(s.second-f.second,2));
            if (width < 20 && width > 5) {
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
        if (dist < 50) {
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

    ros::Subscriber sub = nh.subscribe("/camera/depth/image_rect_raw", 1000, colorCallback);
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
