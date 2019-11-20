#ifndef DETECTING_H
#define DETECTING_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#define PI      3.141592
#define R_ANG   1.570796    // Reference angle[rad]
#define R_DIS   0.4         // Reference distance[m]
#define Kw      0.6         // Gain of radian velocity
#define Kv      0.6         // Gain of distance velocity
#define Kl      0.7         // Weights for updating goal values
#define R_VEL   0.2
#define F_VEL   0.2

using namespace std;

void Association();         // Update to the nearest value within 1.0m of existing value
void Identification();      // Update data only when a person is detected near 0.4m ahead
void GettingHuman();        // When a person is detected
void LostHuman();           // When a person is not detected

geometry_msgs::Twist velOutput;

class DATA{
public:
    double               width;
    pair<double, double> start;
    pair<double, double> finish;
};

class Person {
public:
    bool        valid;
    class DATA  leg1;
    class DATA  leg2;
    pair<double, double> goal;
    Person() { valid = false; goal=make_pair(0,0); }
};

int GetMin(vector<pair<double, double> > list)
{
    double  min     = 99999;
    int     index   = -1;
    for (vector<int>::size_type i = 0; i < list.size(); i++)
    {
        double dist = sqrt(pow(list[i].first,2) + pow(list[i].second,2));
        if (min > dist) {
            min = dist;
            index = i;
        }
    }
    return index;
}

int GetMin(vector<pair<int, pair<double, double> > > list)
{
    double  min     = 99999;
    int     index   = -1;
    for (vector<int>::size_type i = 0; i < list.size(); i++)
    {
        double dist = sqrt(pow(list[i].second.first,2) + pow(list[i].second.second,2));
        if (min > dist) {
            min = dist;
            index = i;
        }
    }
    return index;
}

class Person    p;

// goal<x,y>
vector<pair<double, double> > p_list;                   // List of people near 0.3m

// index matched clustered data, goal<x,y>
vector<pair<int, pair<double, double> > > total_p;      // List of total people within 1.5m

// width, start<x,y>, finish<x,y>
vector<pair<double, pair<pair<double, double>, pair<double, double> > > > clustered;    // clustered data
pair<pair<double, double>, pair<double, double> > c1;

// scan data convert to <x,y>
vector<pair<double,double> >            raw_data;       // Coordinated data
pair<double,double>                     s;
pair<double,double>                     f;
pair<int, pair<double,  double> >       p1;

double  dist;
bool    start_flag = true;
int     err_cnt = 0;

#endif // DETECTING_H
