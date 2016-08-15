#include "utils.h"
#include <cmath>

// moving average filter
MAVFilter::MAVFilter(int ws) {
    setWindowSize(ws);
}

void MAVFilter::setWindowSize(int ws) {
    reset();
    window_size = ws;
}

void MAVFilter::dataIn(double d) {
    assert(data.size() <= window_size);
    if (data.size() == window_size)
            data.pop_back();
    data.push_front(d);
}

double MAVFilter::dataOut() {
    double sum;

    for (std::deque<double>::iterator it = data.begin(); it != data.end(); it++)
        sum = sum + *it;

    return sum / data.size();
}

void MAVFilter::reset() {
    data.clear();
}

// contact detect
void ContactDetect::init(int ws, double interval) {
    prev_sep = cur_sep = 0;
    sepFilter.setWindowSize(ws);
    this->interval = interval;
    tstart = ros::Time::now();
}

bool ContactDetect::checkContact(double sep) {
    sepFilter.dataIn(sep);
    cur_sep = sepFilter.dataOut();

    tnow = ros::Time::now();
    dt = 1e-9 * (tnow - tstart).nsec;
    double sep_diff = 1000;
    if (dt >= 0.2) {
        sep_diff = std::abs(cur_sep - prev_sep);
        prev_sep = cur_sep;
        tstart = tnow;
//        std::cerr << "sep diff " << sep_diff << std::endl;
    }
    if (sep_diff < 0.001)
        return true;

    return false;
}
