#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <ros/ros.h>

extern double primitive_default_close_level;
extern double primitive_default_squeeze_level;
extern double maintain_kp_pos;
extern double maintain_kd_pos;
extern double maintain_kp_rot;
extern double maintain_kd_rot;
extern double contact_window_size;
extern double contact_interval;

void initParameters(ros::NodeHandle& nh);
#endif // PARAMETERS_H
