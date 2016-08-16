#include "executive.h"
#include <eigen_conversions/eigen_kdl.h>
#include "parameters.h"

CartMaintainController::CartMaintainController(Executive *ce, Primitive *primitive, double kp_pos, double kd_pos, double kp_rot, double kd_rot)
    : ControlPrimitiveBase(ce, primitive) {
    control_torque.resize(DOF_JOINTS);
    this->kp_pos = kp_pos;
    this->kd_pos = kd_pos;
    this->kp_rot = kp_rot;
    this->kd_rot = kd_rot;
}

void CartMaintainController::updateControlTorque() {
    for (int i=0; i<control_torque.size(); i++)
        control_torque[i] = 0;

    Eigen::Matrix3d lamda_stiffness, lamda_damping;
    lamda_stiffness = Eigen::Matrix3d::Zero();
    lamda_stiffness(1,1) = kp_pos; lamda_stiffness(2,2) = kp_pos;
    lamda_damping = Eigen::Matrix3d::Zero();
    lamda_damping(1,1) = kd_pos; lamda_damping(2,2) = kd_pos;

    Eigen::Matrix3d Q;
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            Q(i,j) = primitive->prf.M(i,j);

    stiffness_pos = Q * lamda_stiffness * Q.transpose();
    damping_pos = Q * lamda_damping * Q.transpose();

    computeControlTorqueVF(cntrl_vf0);
    computeControlTorqueVF(cntrl_vf1);
}

void CartMaintainController::computeControlTorqueVF(std::vector<ControlOnePatch>& vf) {
    for (std::vector<ControlOnePatch>::iterator it = vf.begin(); it != vf.end(); ++it) {
        ControlOnePatch& pic = *it;

        if (pic.patch->njoints == 0)
            continue;

        std::map<std::string, PatchInfo>::iterator pit = patchInfo.find(pic.patch->name);
        if (pit == patchInfo.end()) {
            std::cerr << "No patch in the map" << std::endl;
            assert(0);
        }
        PatchInfo& pinfo = (*pit).second;

        pic.setStiffnessOrientation(kp_rot);
        pic.setDampingOrientation(kd_rot);
        pic.setStiffnessPosition(stiffness_pos);
        pic.setDampingPosition(damping_pos);

        pic.ictrl.Update_e(pic.patch->kdl_joint_pos_cur);
        pic.ictrl.GetOutput_e(pic.control_torque);

//        std::cerr << pic.control_torque << std::endl;

        for (int i=0; i<pic.patch->njoints; i++) {
            if (boost::math::sign(control_torque[pinfo.joint_map[i]]) == boost::math::sign(pic.control_torque(i,0))) {
                if (boost::math::sign(pic.control_torque(i,0)) >= 0)
                    control_torque[pinfo.joint_map[i]] = std::max(control_torque[pinfo.joint_map[i]], pic.control_torque(i,0));
                else
                    control_torque[pinfo.joint_map[i]] = std::min(control_torque[pinfo.joint_map[i]], pic.control_torque(i,0));
            }
            else
                control_torque[pinfo.joint_map[i]] = control_torque[pinfo.joint_map[i]] + pic.control_torque(i,0);
//            control_torque[pinfo.joint_map[i]] = pic.control_torque(i,0);
        }
    }
}

void CartMaintainController::setTarget() {
    setTargetVF(cntrl_vf0);
    setTargetVF(cntrl_vf1);
}

void CartMaintainController::setTargetVF(std::vector<ControlOnePatch>& vf) {
    for (std::vector<ControlOnePatch>::iterator it = vf.begin(); it != vf.end(); ++it) {
        ControlOnePatch& pic = *it;

        if (pic.patch->njoints == 0)
            continue;

        Eigen::Affine3d target_transform;
        tf::transformKDLToEigen(pic.patch->patch_pose, target_transform);
        pic.ictrl.SetTarget_e(target_transform.translation(), target_transform.rotation());
    }
}

