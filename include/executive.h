#ifndef PRIMITIVE_CONTROLLER_H
#define PRIMITIVE_CONTROLLER_H

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <ImpedanceController.h>

#include "primitive.h"
#include "utils.h"

#include <dynamic_reconfigure/server.h>
#include <allegro_opposition_primitives/my_dyn_paramsConfig.h>

class CloseController;
enum eControllerState {
    NONE,
    PRESHAPE,
    CLOSE,
    MAINTAIN,
    SQUEEZE
};

class ControlPrimitiveBase;
class SqueezeController;
class CartMaintainController;
class JointMaintainController;
class GravCompController;
class PrimitiveController;

class Executive {
public:
    Executive() {}
    ~Executive();

    void initController();
    void updateController();
    void setJointStateCallback(const sensor_msgs::JointState &msg);
    void controlCommandCallback(const std_msgs::String &msg);
    void broadcastPatchTF(std::vector<Patch>& patches);
    bool detectContact();

    void printPrimitiveDef(const std::string& name, XmlRpc::XmlRpcValue& def);

    // ROS stuff
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub;
    ros::Subscriber control_cmd_sub;
    ros::Publisher control_cmd_pub;
    ros::Publisher cmd_pub;
    ros::Publisher contact_pub;
    ros::Publisher lib_cmd_pub;

    // Allegro state and its synchronization mutex
    sensor_msgs::JointState current_joint_state;
    bool js_ready;
    boost::mutex *mutex;
    KDL::Tree my_tree;

    // ROS Time

    // Controllers
    GravCompController *gravcomp;
    std::vector<PrimitiveController*> controllers;
    std::vector<double> preshape;
    eControllerState cntrl_state;

    // control command
    sensor_msgs::JointState cntrl_joint_state;

private:
    std::string graspName;

    bool initPrimitiveDefinition(std::string primitive_name, PrimitiveDefinition& def);
    bool populateGrasp(std::string grasp_name);
    void transformPatchNames(std::vector<std::string>& names);
    void reconfigureCallback(allegro_opposition_primitives::my_dyn_paramsConfig& config, uint32_t level);
    void initGravCompHack();
    void goToHome();

    void enableJointControl() { cntrl_joint_state.position.resize(DOF_JOINTS); cntrl_joint_state.effort.clear(); }
    void disableJointControl() { cntrl_joint_state.position.clear(); cntrl_joint_state.effort.resize(DOF_JOINTS); }
    bool inJointControlMode() { return cntrl_joint_state.position.size() == DOF_JOINTS; }

    dynamic_reconfigure::Server<allegro_opposition_primitives::my_dyn_paramsConfig> *dyn_reconf_server;
    dynamic_reconfigure::Server<allegro_opposition_primitives::my_dyn_paramsConfig>::CallbackType dyn_reconf_callback_binded;

};

class ControlOnePatch {
public:
    ControlOnePatch(Patch *p);
    void setStiffnessPosition(double kp);
    void setStiffnessPosition(Eigen::Matrix3d& kp);
    void setDampingPosition(double kd);
    void setDampingPosition(Eigen::Matrix3d& kd);
    void setStiffnessOrientation(double kp);
    void setDampingOrientation(double kd);

    ImpedanceController ictrl;
    Eigen::Matrix<double, Eigen::Dynamic, 1> control_torque;
    Eigen::Matrix<double, Eigen::Dynamic, 1> err;
    Patch *patch;
};

class ControlPrimitiveBase {
public:
    ControlPrimitiveBase(Executive* ce, Primitive *primitive);
    virtual void updateControlTorque() = 0;
    virtual void setTarget() = 0;

private:
    void initPatchControl(std::vector<ControlOnePatch>& patch_cntrl, std::vector<Patch>& vf);

public:
    std::vector<ControlOnePatch> cntrl_vf0, cntrl_vf1;
    Executive* ce;
    std::vector<double> control_torque;
    Primitive *primitive;

    // deubug
    tf::Transform ftr;
};

class SqueezeController : public ControlPrimitiveBase {
public:
    SqueezeController(Executive* ce, Primitive *primitive);
    void updateControlTorque();
    void setTarget() {}
    void setTargetForSqueeze() { targetDistance = &squeezeLevel; }
    void setTargetForClose() { targetDistance = &closeLevel; }

private:
    void computeControlTorqueVF(std::vector<ControlOnePatch>& vf, const KDL::Vector& ovec);

public:
    double *targetDistance;
    double squeezeLevel;
    double closeLevel;
};

class CartMaintainController : public ControlPrimitiveBase {
public:
    CartMaintainController(Executive *ce, Primitive *primitive, double kp_pos, double kd_pos, double kp_rot, double kd_rot);
    void updateControlTorque();
    void setTarget();

private:
    void computeControlTorqueVF(std::vector<ControlOnePatch>& vf);
    void setTargetVF(std::vector<ControlOnePatch>& vf);

    double kp_pos, kd_pos, kp_rot, kd_rot;
    Eigen::Matrix3d stiffness_pos, damping_pos;
};

class GravCompController : public ControlPrimitiveBase {
public:
    GravCompController(Executive *ce, Primitive *primitive=(Primitive *)NULL);
    void updateControlTorque() {}
    void setTarget();
};

class PrimitiveController {
public:
    PrimitiveController(Executive *ce, Primitive *primitive, double mix_factor);
    void updateController();
    std::vector<double>& getControlTorque() { return control_torque; }
    void setControlState(eControllerState state);

    void setSqueezeLevel(double level) { squeeze->squeezeLevel = level; }
    void setCloseLevel(double level) { squeeze->closeLevel = level; }
    void setMixFactor(double mix_factor) {this->mix_factor = mix_factor;}

private:
    Executive *ce;
    Primitive *primitive;
    std::vector<double> control_torque;
    double mix_factor;

    ContactDetect contact;
    CartMaintainController *maintain;
    SqueezeController *squeeze;
    eControllerState cntrl_state;

    friend class Executive;
};
#endif // PRIMITIVE_CONTROLLER_H
