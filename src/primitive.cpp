#include "primitive.h"
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <Eigen/Eigen>

///////////////////////////////
/// Patch
///////////////////////////////
Patch::Patch(std::string patch_name, double importance, KDL::Tree& hand_tree) {
    std::map<std::string, PatchInfo>::iterator it = patchInfo.find(patch_name);
    if (it == patchInfo.end()) {
        std::cerr << "No patch in the map" << std::endl;
        assert(0);
    }
    PatchInfo& pinfo = (*it).second;

    hand_tree.getChain("palm_link", pinfo.name, kdlchain);
    njoints = kdlchain.getNrOfJoints();
    for (int i=0; i<pinfo.patch_extension.size(); i++)
        kdlchain.addSegment(pinfo.patch_extension[i]);
    std::cerr << patch_name << " chain has" << njoints << "joints" << std::endl;
    std::cerr << patch_name << " chain has" << kdlchain.getNrOfSegments() << "segments" << std::endl;
    kdl_joint_pos_cur.resize(njoints);
    this->name = patch_name;
    this->importance = importance;

    std::cerr << this->name << " " << this->importance << std::endl;
}

void Patch::updateState(sensor_msgs::JointState &allegro_state) {
    std::map<std::string, PatchInfo>::iterator it = patchInfo.find(name);
    if (it == patchInfo.end()) {
        std::cerr << "No patch in the map" << std::endl;
        assert(0);
    }
    PatchInfo& pinfo = (*it).second;

    for (int i=0; i<njoints; i++)
        kdl_joint_pos_cur(i) = allegro_state.position[pinfo.joint_map[i]];

    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(kdlchain);
    if (fksolver.JntToCart(kdl_joint_pos_cur, patch_pose)>=0) {}
    else {
        std::cerr << "Could not calculate forward kinematics for chain f0. Can't go on" << std::endl;
        exit(0);
    }
}


///////////////////////////////
/// Primitive
///////////////////////////////
void PrimitiveDefinition::print() {
    std::cerr << "vf 0: ";
    for (int i=0; i <vf0.size(); i++)
        std::cerr << vf0[i] << " ";
    std::cerr << std::endl;
    std::cerr << "  imp: ";
    for (int i=0; i <fdist0.size(); i++)
        std::cerr << fdist0[i] << " ";
    std::cerr << std::endl;
    std::cerr << "  fthresh: " << focus_thresh_0 << std::endl;

    std::cerr << "vf 1: ";
    for (int i=0; i <vf1.size(); i++)
        std::cerr << vf1[i] << " ";
    std::cerr << std::endl;
    std::cerr << "  imp: ";
    for (int i=0; i <fdist1.size(); i++)
        std::cerr << fdist1[i] << " ";
    std::cerr << std::endl;
    std::cerr << "  fthresh: " << focus_thresh_1 << std::endl;
}

Primitive::Primitive(PrimitiveDefinition& pdef, KDL::Tree& hand_tree) {
    active_joints.resize(DOF_JOINTS);
    for (int i=0; i<DOF_JOINTS; i++)
        active_joints[i] = false;

    // create the virtual finger patch groups
    for (int i=0; i<pdef.vf0.size(); i++) {
        Patch p = Patch(pdef.vf0[i], pdef.fdist0[i], hand_tree);
        vf0.push_back(p);
        updateActiveJoints(p);
    }

    for (int i=0; i<pdef.vf1.size(); i++) {
        Patch p = Patch(pdef.vf1[i], pdef.fdist1[i], hand_tree);
        vf1.push_back(p);
        updateActiveJoints(p);
    }

    focus_thresh_0 = pdef.focus_thresh_0;
    focus_thresh_1 = pdef.focus_thresh_1;

    name = pdef.name;
    preshape = pdef.preshape;
}

void Primitive::updateActiveJoints(Patch& p) {
    std::map<std::string, PatchInfo>::iterator it = patchInfo.find(p.name);
    if (it == patchInfo.end()) {
        std::cerr << "No patch in the map" << std::endl;
        assert(0);
    }
    PatchInfo& pinfo = (*it).second;

    for (int i=0; i<pinfo.joint_map.size(); i++)
        active_joints[pinfo.joint_map[i]] = true;
}

void Primitive::printActiveJoints() {
    std::cerr << name << " : ";
    for (int i=0; i<DOF_JOINTS; i++)
        if (active_joints[i]) std::cerr << i << " ";
    std::cerr << std::endl;
}

void Primitive::updateVirtualFingers(sensor_msgs::JointState &allegro_state) {
    focusCompute_init(focus_0);
    for(std::vector<Patch>::iterator it = vf0.begin(); it!=vf0.end(); ++it) {
       (*it).updateState(allegro_state);
        focusCompute_add(focus_0, *it, focus_thresh_0);
    }
    focusCompute_terminate(focus_0);

    focusCompute_init(focus_1);
    for(std::vector<Patch>::iterator it = vf1.begin(); it!=vf1.end(); ++it) {
       (*it).updateState(allegro_state);
        focusCompute_add(focus_1, *it, focus_thresh_1);
    }
    focusCompute_terminate(focus_1);

    ovec = focus_1.p - focus_0.p;
    ovec_norm = sqrt(pow(ovec(0),2) + pow(ovec(1),2) + pow(ovec(2),2));
    ovec = ovec / ovec_norm;

    // broadcast the foci
    double x, y, z, w;
    static tf::TransformBroadcaster br;
    ftr.setOrigin(tf::Vector3(focus_0.p[0], focus_0.p[1], focus_0.p[2]));
    focus_0.M.GetQuaternion(x, y, z, w);
    ftr.setRotation(tf::Quaternion(x, y, z, w));
    br.sendTransform(tf::StampedTransform(ftr, ros::Time::now(), "palm_link", "focus_0"));

    ftr.setOrigin(tf::Vector3(focus_1.p[0], focus_1.p[1], focus_1.p[2]));
    focus_1.M.GetQuaternion(x, y, z, w);
    ftr.setRotation(tf::Quaternion(x, y, z, w));
    br.sendTransform(tf::StampedTransform(ftr, ros::Time::now(), "palm_link", "focus_1"));

    // compute the prf with ovec as +x
//    Eigen::Matrix<double, 1, 3> ovec_xrow(ovec[0], ovec[1], ovec[2]);
//    Eigen::FullPivLU<Eigen::MatrixXd> lu(ovec_xrow);
//    Eigen::MatrixXd x_ns = lu.kernel();
//    Eigen::Matrix<double, 3, 1> ovec_y(x_ns(0,0), x_ns(1,0), x_ns(2,0));
//    ovec_y.normalize();
//    Eigen::Matrix<double, 3, 1> ovec_x = ovec_xrow.transpose();
//    Eigen::Matrix<double, 3, 1> ovec_z = ovec_x.cross(ovec_y);
//    KDL::Vector prf_pos = (focus_0.p + focus_1.p) / 2;
//    KDL::Rotation prf_rot = KDL::Rotation(ovec_x(0,0), ovec_x(1,0), ovec_x(2,0), ovec_y(0,0), ovec_y(1,0), ovec_y(2,0), ovec_z(0,0), ovec_z(1,0), ovec_z(2,0));
//    KDL::Frame prf = KDL::Frame(prf_rot, prf_pos);

    KDL::Vector axis = focus_0.M.UnitX()*ovec;
    double angle = acos(dot(focus_0.M.UnitX(),ovec));
    KDL::Rotation target_r = KDL::Rotation::Rot(axis, angle) * focus_0.M;
    KDL::Vector target_p = (focus_0.p + focus_1.p) / 2;
    prf = KDL::Frame(target_r, target_p);

    ftr.setOrigin(tf::Vector3(prf.p[0], prf.p[1], prf.p[2]));
    prf.M.GetQuaternion(x, y, z, w);
    ftr.setRotation(tf::Quaternion(x, y, z, w));
    br.sendTransform(tf::StampedTransform(ftr, ros::Time::now(), "palm_link", "prf"));
}

void Primitive::focusCompute_init(KDL::Frame& focus) {
    focus_compute_npatch = 0;
    focus.p = KDL::Vector::Zero();
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            focus.M(i,j) = 0;
}

void Primitive::focusCompute_add(KDL::Frame& focus, Patch& patch, double threshold) {
    if (patch.importance < threshold)
        return;

    focus.p = focus.p + patch.patch_pose.p;
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            focus.M(i,j) = focus.M(i,j) + patch.patch_pose.M(i,j);

    focus_compute_npatch++;
}

void Primitive::focusCompute_terminate(KDL::Frame& focus) {
    focus.p = focus.p / focus_compute_npatch;
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            focus.M(i,j) = focus.M(i,j) / focus_compute_npatch;
}
