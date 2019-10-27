#ifndef DETECTING_DEPTH_H
#define DETECTING_DEPTH_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

#define PI      3.141592
#define R_VEL   0.2         // rotate velocity
#define F_VEL   0.2         // forward velocity
#define HEIGHT  480
#define WIDTH   640
#define LINE    239
#define STEP    1280

using namespace std;

void GettingHuman();
void LostHuman();

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

class Person    p;

// index matched clustered data, goal<x,y>
vector<pair<int, pair<double, double> > > total_p;

// width, start<x,y>, finish<x,y>
vector<pair<double, pair<pair<double, double>, pair<double, double> > > > clustered;    // clustered data
pair<pair<double, double>, pair<double, double> > c1;

// scan data convert to <x,y>
vector<pair<double,double> >            raw_data;
pair<double,double>                     s;
pair<double,double>                     f;
pair<int, pair<double,  double> >       p1;

double  dist;
bool    start_flag = true;

#endif // DETECTING_DEPTH_H
