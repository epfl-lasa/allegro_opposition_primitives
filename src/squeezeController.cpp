#include "primitiveController.h"
#include <eigen_conversions/eigen_kdl.h>
#include "parameters.h"

SqueezeController::SqueezeController(Executive* ce, Primitive *primitive)
    : ControlPrimitiveBase(ce, primitive) {
    control_torque.resize(DOF_JOINTS);
    targetDistance = &closeLevel;
    squeezeLevel = primitive_default_squeeze_level;
    closeLevel = primitive_default_close_level;
}

void SqueezeController::updateControlTorque() {
    for (int i=0; i<control_torque.size(); i++)
        control_torque[i] = 0;

    computeControlTorqueVF(cntrl_vf0, primitive->ovec);
    computeControlTorqueVF(cntrl_vf1, -(primitive->ovec));
}

void SqueezeController::computeControlTorqueVF(std::vector<ControlOnePatch>& vf, const KDL::Vector& ovec) {
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

        // set target
        KDL::Vector target_p = pic.patch->patch_pose.p + ovec * (*targetDistance) * pic.patch->importance;
        KDL::Vector axis = pic.patch->patch_pose.M.UnitX()*ovec;
        double angle = acos(dot(pic.patch->patch_pose.M.UnitX(),ovec));
        KDL::Rotation target_r = KDL::Rotation::Rot(axis, angle) * pic.patch->patch_pose.M;
        KDL::Frame target = KDL::Frame(target_r, target_p);

        // control torque for patch kinematic chain to achieve target
        Eigen::Affine3d target_transform;
        tf::transformKDLToEigen(target, target_transform);
        pic.ictrl.SetTarget_e(target_transform.translation(), target_transform.rotation());
        pic.ictrl.Update_e(pic.patch->kdl_joint_pos_cur);
        pic.ictrl.GetOutput_e(pic.control_torque);

        // accumulate the torques for all the patches
        for (int i=0; i<pic.patch->njoints; i++) {
            if (boost::math::sign(control_torque[pinfo.joint_map[i]]) == boost::math::sign(pic.control_torque(i,0))) {
                if (boost::math::sign(pic.control_torque(i,0)) >= 0)
                    control_torque[pinfo.joint_map[i]] = std::max(control_torque[pinfo.joint_map[i]], pic.control_torque(i,0));
                else
                    control_torque[pinfo.joint_map[i]] = std::min(control_torque[pinfo.joint_map[i]], pic.control_torque(i,0));
            }
            else
                control_torque[pinfo.joint_map[i]] = control_torque[pinfo.joint_map[i]] + pic.control_torque(i,0);
        }

        // publish focus and targets for debug
        // patch
        double x, y, z, w;
        static tf::TransformBroadcaster br;
        ftr.setOrigin(tf::Vector3(pic.patch->patch_pose.p[0], pic.patch->patch_pose.p[1], pic.patch->patch_pose.p[2]));
        pic.patch->patch_pose.M.GetQuaternion(x, y, z, w);
        ftr.setRotation(tf::Quaternion(x, y, z, w));
        //br.sendTransform(tf::StampedTransform(ftr, ros::Time::now(), "palm_link", pic.patch->name));


        // target
        ftr.setOrigin(tf::Vector3(target.p[0], target.p[1], target.p[2]));
        target.M.GetQuaternion(x, y, z, w);
        ftr.setRotation(tf::Quaternion(x, y, z, w));
        //br.sendTransform(tf::StampedTransform(ftr, ros::Time::now(), "palm_link", pic.patch->name + "_T"));
    }
}
