#ifndef UTILS_H
#define UTILS_H

#include <deque>
#include <ros/ros.h>

class MAVFilter {

public:
    MAVFilter(int window_size = 10);
    void setWindowSize(int ws);
    void dataIn(double);
    double dataOut();
    void reset();

private:
    std::deque<double> data;
    int window_size;
};

class ContactDetect {
public:
    ContactDetect() {}
    void init(int window_size, double interval);
    bool checkContact(double separation);

private:
    MAVFilter sepFilter;
    double cur_sep, prev_sep;
    ros::Time tstart;
    ros::Time tnow;
    double dt;
    double interval;
};

#endif // UTILS_H
