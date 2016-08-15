#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/tree.hpp>

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "patchInfo.h"
#include "utils.h"

#include <string>
#include <map>
#include <vector>

class Patch {
public:
    Patch(std::string name, double importance, KDL::Tree& hand_tree);
    void updateState(sensor_msgs::JointState& allegro_state);

    // patch control
    KDL::Chain kdlchain;
    int njoints;
    KDL::JntArray kdl_joint_pos_cur;
    KDL::Frame patch_pose;
    std::string name;
    double importance;
};

class PrimitiveDefinition {
public:
    PrimitiveDefinition() {}
    std::string name;
    std::vector<std::string> vf0;
    std::vector<double> fdist0;
    double focus_thresh_0;
    std::vector<std::string> vf1;
    std::vector<double> fdist1;
    double focus_thresh_1;
    std::vector<double> preshape;

    void print();
};

class Primitive {
public:
    Primitive(PrimitiveDefinition& pdef, KDL::Tree& hand_tree);
    void updateVirtualFingers(sensor_msgs::JointState& allegro_state);

private:
    void focusCompute_init(KDL::Frame& focus);
    void focusCompute_add(KDL::Frame& focus, Patch& patch, double threshold);
    void focusCompute_terminate(KDL::Frame& focus);
    void updateActiveJoints(Patch& p);
    void printActiveJoints();

public:
    // VF state
    std::vector<Patch> vf0, vf1;

    KDL::Frame focus_0, focus_1, prf;
    double focus_thresh_0, focus_thresh_1;
    std::vector<bool> active_joints;
    std::vector<double> preshape;
    int focus_compute_npatch;
    tf::Transform ftr;

    KDL::Vector ovec;
    double ovec_norm;
    std::string name;    

    friend class PrimitiveController;
};

#endif // PRIMITIVE_H
