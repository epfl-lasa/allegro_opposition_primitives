#include "parameters.h"

double primitive_default_close_level        = 0.7;
double primitive_default_squeeze_level      = 0.7;
double maintain_kp_pos                      = 100;
double maintain_kd_pos                      = 70;
double maintain_kp_rot                      = 0;
double maintain_kd_rot                      = 0;
double contact_window_size                  = 30;
double contact_interval                     = 0.2;

void initParameters(ros::NodeHandle& nh) {
    nh.getParam(ros::this_node::getName() + "/primitive_default_close_level", primitive_default_close_level);
    nh.getParam(ros::this_node::getName() + "/primitive_default_squeeze_level", primitive_default_squeeze_level);
    nh.getParam(ros::this_node::getName() + "/maintain_kp_pos", maintain_kp_pos);
    nh.getParam(ros::this_node::getName() + "/maintain_kd_pos", maintain_kd_pos);
    nh.getParam(ros::this_node::getName() + "/maintain_kp_rot", maintain_kp_rot);
    nh.getParam(ros::this_node::getName() + "/maintain_kd_rot", maintain_kd_rot);
    nh.getParam(ros::this_node::getName() + "/contact_window_size", contact_window_size);
    nh.getParam(ros::this_node::getName() + "/contact_interval", contact_interval);
}
