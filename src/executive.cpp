#include "executive.h"
#include "primitive.h"
#include "parameters.h"

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "std_msgs/String.h"
#include <boost/math/special_functions/sign.hpp>

#include <XmlRpcException.h>

#include <iostream>
#include <string>
#include <exception>

const std::string JOINT_STATE_TOPIC = "allegroHand/joint_states";
const std::string DESIRED_STATE_TOPIC = "allegroHand/joint_cmd";
const std::string CONTACT_TOPIC = "allegroHand/contact_detect";
const std::string CONTROL_TOPIC = "allegroHand/primitive_control_cmd";
const std::string LIB_CMD_TOPIC = "allegroHand/lib_cmd";
const std::string PRIMITIVE_PREFIX = "/primitives";
const std::string GRASP_PREFIX = "/grasps";
const std::string PRESHAPE_PREFIX = "/preshapes";


void Executive::initController() {
    initParameters(nh);

    std::string robot_description;
    // get the urdf description from the parameter server.
    if (nh.getParam("robot_description", robot_description)) {}
    else {
        std::cerr << "could not load robot_description";
        exit(0);
    }

    // get kdl tree    
    int nJoints;
    if ( !kdl_parser::treeFromString(robot_description, my_tree) ) {
        ROS_FATAL("Failed to construct kdl tree");
        exit(0);
    } else {
        nJoints = my_tree.getNrOfJoints();
        std::cerr << "= There are " << my_tree.getNrOfJoints() << " joints in my robot" << std::endl;
    }

    for (KDL::SegmentMap::const_iterator it = my_tree.getSegments().begin(); it != my_tree.getSegments().end(); ++it)
        std::cerr << it->second.segment.getName() << std::endl;

    // common controllers
    gravcomp = new GravCompController(this);
    cntrl_joint_state.effort.resize(DOF_JOINTS);
    for (int i=0; i < DOF_JOINTS; i++)
        cntrl_joint_state.effort[i] = 0;    
    cntrl_state = NONE;

    // state synchronization
    mutex = new boost::mutex();
    js_ready = false;

    // Use tcpNoDelay to achieve stable control loop.
    joint_state_sub = nh.subscribe(JOINT_STATE_TOPIC, 1, &Executive::setJointStateCallback, this, ros::TransportHints().tcpNoDelay());
    control_cmd_sub = nh.subscribe(CONTROL_TOPIC, 1, &Executive::controlCommandCallback, this);
    cmd_pub = nh.advertise<sensor_msgs::JointState>(DESIRED_STATE_TOPIC, 3);
    contact_pub = nh.advertise<std_msgs::String>(CONTACT_TOPIC, 3);
    lib_cmd_pub = nh.advertise<std_msgs::String>(LIB_CMD_TOPIC, 3);
    control_cmd_pub = nh.advertise<std_msgs::String>(CONTROL_TOPIC, 3);

    initGravCompHack();

    enableJointControl();
    graspName = "";

    // Dyn parameters callback  (must be done last because we will receive a callback as soon as the server is started
    dyn_reconf_server          = new dynamic_reconfigure::Server<allegro_opposition_primitives::my_dyn_paramsConfig>();
    dyn_reconf_callback_binded = boost::bind(&Executive::reconfigureCallback, this, _1, _2);
    dyn_reconf_server->setCallback(dyn_reconf_callback_binded);
}

void Executive::initGravCompHack() {
    std::vector<double> grav_comp_pose;
    if (nh.getParam(ros::this_node::getName() + "/grav_comp_pose", grav_comp_pose)) {}
    else {
        std::cerr << "could not init grav comp hack" << std::endl;
        exit(0);
    }

    bool ready = false;
    while (!ready) {
        mutex->lock();
        ready = js_ready;
        mutex->unlock();
        ros::spinOnce();
    }

    enableJointControl();
    cntrl_joint_state.position = grav_comp_pose;
    std_msgs::String cmd;
    cmd.data = "pdControl";
    for (int i=0; i<10; i++)
        lib_cmd_pub.publish(cmd);
    for (int i=0; i<10; i++)
          cmd_pub.publish(cntrl_joint_state);                    
    ros::Duration(3).sleep();
    for (int i=0; i<10; i++)
        ros::spinOnce();
    gravcomp->setTarget();
    disableJointControl();

    std::cerr << "Grav comp hack: ";
    for (int i=0; i<DOF_JOINTS; i++)
        std::cerr << gravcomp->control_torque[i] << " ";
    std::cerr << std::endl;

}


bool Executive::initPrimitiveDefinition(std::string primitive_name, PrimitiveDefinition& def) {
    XmlRpc::XmlRpcValue primitive;
    std::string name = PRIMITIVE_PREFIX + "/" + primitive_name;
    std::string preshape;

    try {
        if (!nh.getParam(name, primitive)) throw XmlRpc::XmlRpcException("malformed primitive defn: unknown primitive");
        if (!nh.getParam(name + "/vf0_patch", def.vf0)) throw XmlRpc::XmlRpcException("malformed primitive defn: vf0 patch"); else transformPatchNames(def.vf0);
        if (!nh.getParam(name + "/vf0_imp", def.fdist0)) throw XmlRpc::XmlRpcException("malformed primitive defn: vf0 imp");
        if (!nh.getParam(name + "/vf0_focus", def.focus_thresh_0)) throw XmlRpc::XmlRpcException("malformed primitive defn: vf0 focus");
        if (!nh.getParam(name + "/vf1_patch", def.vf1)) throw XmlRpc::XmlRpcException("malformed primitive defn: vf1 patch"); else transformPatchNames(def.vf1);
        if (!nh.getParam(name + "/vf1_imp", def.fdist1)) throw XmlRpc::XmlRpcException("malformed primitive defn: vf1 imp");
        if (!nh.getParam(name + "/vf1_focus", def.focus_thresh_1)) throw XmlRpc::XmlRpcException("malformed primitive defn: vf1 focus");

        def.name = primitive_name;
//        def.print();
    }
    catch (std::exception& e) {
        std::cerr << "unable to init primitive " << primitive_name << std::endl;
        return false;
    }

    return true;
}

void Executive::transformPatchNames(std::vector<std::string>& names) {
    for(int i=0; i<names.size(); i++) {
        if (patchNameMap.find(names[i]) == patchNameMap.end())
            throw std::exception();
        names[i] = patchNameMap[names[i]];
    }
}

void Executive::goToHome() {
    std_msgs::String cmd;
    cmd.data = "home";
    control_cmd_pub.publish(cmd);
}

bool Executive::populateGrasp(std::string grasp_name) {
    std::cerr << "populateGrasp: " << grasp_name << std::endl;
    if (graspName == grasp_name)
        return true;

    controllers.clear();

    if (grasp_name == "grasp_none") {
        goToHome();
        return false;
    }

    XmlRpc::XmlRpcValue grasp;
    std::string name = GRASP_PREFIX + "/" + grasp_name;
//    std::cerr << "Grasp : " << grasp_name << std::endl;
    std::vector<std::string> primitives;    
    try {
        if (!nh.getParam(name, grasp)) throw XmlRpc::XmlRpcException("no parameter : " + name);
        if (!nh.getParam(name + "/primitives", primitives)) throw XmlRpc::XmlRpcException("malformed grasp defn : no primitives");
        for (int i=0; i<primitives.size(); i++) {
            XmlRpc::XmlRpcValue p = grasp[primitives[i]];
            std::string pname = static_cast<std::string>(p[0]);
            double mix_factor = static_cast<double>(p[1]);

//            std::cerr << "Primitive " << i << ": " << pname << " with " << mix_factor << std::endl;

            PrimitiveDefinition pdef;
            initPrimitiveDefinition(pname, pdef);
            controllers.push_back(new PrimitiveController(this, new Primitive(pdef, my_tree), mix_factor));            
        }
        if (!nh.getParam(name + "/preshape", preshape)) throw XmlRpc::XmlRpcException("malformed grasp defn: no preshape");
    }
    catch (XmlRpc::XmlRpcException& e) {
        std::cerr << "unable to init grasp " << grasp_name << " reason: " << e.getMessage() << std::endl;
        goToHome();
        return false;
    }

    graspName = grasp_name;
    return true;
}

void Executive::reconfigureCallback(allegro_opposition_primitives::my_dyn_paramsConfig& config, uint32_t level) {
    // std::cerr << "Grasp: " << config.grasp << std::endl;
    std_msgs::String cmd;
    cmd.data = "grasp " + config.grasp;
    controlCommandCallback(cmd);

    //std::cerr << "Primitive: " << config.primitive << std::endl;
//    std::cerr << "Mix factor: " << config.mix_factor << std::endl;
//    std::cerr << "close_level: " << config.close_level << std::endl;
//    std::cerr << "squeeze_level: " << config.squeeze_level << std::endl;

    mutex->lock();
    if (config.primitive >= controllers.size()) {
        for (std::vector<PrimitiveController*>::iterator it = controllers.begin(); it != controllers.end(); ++it) {
            (*it)->setSqueezeLevel(config.squeeze_level);
            (*it)->setCloseLevel(config.close_level);
        }
    }
    else {
        controllers[config.primitive]->setSqueezeLevel(config.squeeze_level);
        controllers[config.primitive]->setCloseLevel(config.close_level);
    }
    mutex->unlock();

    //    std::cerr << "Do close: " << (config.close ? "Yes" : "No") << std::endl;
    if (config.close) {
        std_msgs::String cmd;
        cmd.data = "close";
        controlCommandCallback(cmd);
    } else {
        std_msgs::String cmd;
        cmd.data = "open";
        controlCommandCallback(cmd);
    }
}

void Executive::controlCommandCallback(const std_msgs::String &msg) {
    std::cerr << "[EXECUTIVE]: received command -- " << msg.data << std::endl;
    mutex->lock();
    if (msg.data == "home" && (cntrl_state == NONE || cntrl_state == PRESHAPE)) {
        std::cerr << "[EXECUTIVE]: Heard home" << std::endl;
        controllers.clear();
        enableJointControl();
        graspName = "grasp_none";

        std_msgs::String cmd;
        cmd.data = "home";
        lib_cmd_pub.publish(cmd);

        cntrl_state = NONE;
    }

    if (msg.data.find("grasp ") != std::string::npos && (cntrl_state == PRESHAPE || cntrl_state == NONE)) {
        std::cerr << "[EXECUTIVE]: Heard grasp" << std::endl;
        assert(inJointControlMode());

        std::string grasp = msg.data.substr(6);
        if (!populateGrasp(grasp)) {
            std::cerr << "Unable to populate grasp -- " << grasp << " -- state unchanged" << std::endl;
            mutex->unlock();
            return;
        }

        for (std::vector<PrimitiveController*>::iterator it = controllers.begin(); it != controllers.end(); ++it)
            (*it)->setControlState(PRESHAPE);

        cntrl_state = PRESHAPE;
    }

    if (msg.data == "close" && cntrl_state == PRESHAPE) {
        std::cerr << "[EXECUTIVE]: Heard close" << std::endl;
        disableJointControl();
        for (std::vector<PrimitiveController*>::iterator it = controllers.begin(); it != controllers.end(); ++it)
            (*it)->setControlState(CLOSE);
        cntrl_state = CLOSE;
    }

    if (msg.data == "open" && cntrl_state == CLOSE) {
        std::cerr << "[EXECUTIVE]: Heard open" << std::endl;
        enableJointControl();
        for (std::vector<PrimitiveController*>::iterator it = controllers.begin(); it != controllers.end(); ++it)
            (*it)->setControlState(PRESHAPE);
        cntrl_state = PRESHAPE;
    }

    if (msg.data.find("sq ") != std::string::npos && cntrl_state == CLOSE) {
        double level;
        int pid;
        char *pEnd;

        assert(!inJointControlMode());
        pid = std::strtol(msg.data.substr(3).c_str(), &pEnd, 0);
        level = std::strtod(pEnd, NULL);
        if (pid < controllers.size())
            controllers[pid]->setSqueezeLevel(level);
        else
            for (std::vector<PrimitiveController*>::iterator it = controllers.begin(); it != controllers.end(); ++it)
                (*it)->setSqueezeLevel(level);
    }

    mutex->unlock();
}

Executive::~Executive() {
    delete mutex;
}

void Executive::broadcastPatchTF(std::vector<Patch>& patches) {
    double x, y, z, w;
    static tf::TransformBroadcaster br;
    tf::Transform ftr;

    for (std::vector<Patch>::iterator it = patches.begin(); it != patches.end(); ++it) {
        ftr.setOrigin(tf::Vector3((*it).patch_pose.p[0], (*it).patch_pose.p[1], (*it).patch_pose.p[2]));
        (*it).patch_pose.M.GetQuaternion(x, y, z, w);
        ftr.setRotation(tf::Quaternion(x, y, z, w));
        br.sendTransform(tf::StampedTransform(ftr, ros::Time::now(), "palm_link", (*it).name));
    }
}

void Executive::setJointStateCallback(const sensor_msgs::JointState &msg) {
    mutex->lock();
    current_joint_state = msg;
    js_ready = true;
    mutex->unlock();
}


void Executive::updateController() {
    // update latest joint state.
    mutex->lock();
    if (!js_ready || cntrl_state == NONE) {
        mutex->unlock();
        return;
    }

    for (std::vector<PrimitiveController*>::iterator it = controllers.begin(); it != controllers.end(); ++it) {
        PrimitiveController *c = *it;
        c->primitive->updateVirtualFingers(current_joint_state);
        //broadcastPatchTF(c->primitive->vf0);
        //broadcastPatchTF(c->primitive->vf1);
        c->updateController();
    }

    if (inJointControlMode()) {
        for (int i=0; i<DOF_JOINTS; i++) {
//            cntrl_joint_state.effort[i] = 0;
            cntrl_joint_state.position[i] = preshape[i];
        }
    }
    else {
        //gravcomp->setTarget();
        for (int i=0; i<DOF_JOINTS; i++) {
            cntrl_joint_state.effort[i] = gravcomp->control_torque[i];
            for (int j=0; j<controllers.size(); j++)
                cntrl_joint_state.effort[i] += (controllers[j]->getControlTorque())[i] * controllers[j]->mix_factor;
        }
    }

    cmd_pub.publish(cntrl_joint_state);
    mutex->unlock();
}

////////////////////////
/// PrimitiveController
////////////////////////
PrimitiveController::PrimitiveController(Executive *ce, Primitive *primitive, double mix_factor) {
    this->ce = ce;
    this->primitive = primitive;
    squeeze = new SqueezeController(ce, primitive);
    maintain = new CartMaintainController(ce, primitive, maintain_kp_pos, maintain_kd_pos, maintain_kp_rot, maintain_kd_rot);
    cntrl_state = NONE;
    control_torque.resize(DOF_JOINTS);
    this->mix_factor = mix_factor;
    primitive->printActiveJoints();
}

void PrimitiveController::updateController() {
    bool maintain_active = false;

    for (int i=0; i<control_torque.size(); i++)
        control_torque[i] = 0;

    if (cntrl_state == PRESHAPE || cntrl_state == NONE)
        return;


    squeeze->updateControlTorque();

    if (cntrl_state == CLOSE) {
        bool inContact = contact.checkContact(primitive->ovec_norm);
        std::cerr << "[" << primitive->name << "]: CLOSING " << primitive->ovec_norm << " Contact: " << inContact << std::endl;
        if (inContact) {
            std::cerr << "[" << primitive->name << "]: Finished CLOSE, enter MAINTAIN" << std::endl;
            setControlState(MAINTAIN);
        }
    }

    if (cntrl_state == MAINTAIN) {
        std::cerr << "[" << primitive->name << "]: MAINTAIN with squeeze level " << *(squeeze->targetDistance) << std::endl;
        maintain->updateControlTorque(); maintain_active = true;
    }

    for (int i=0; i<DOF_JOINTS; i++) {
        if (!primitive->active_joints[i])
            continue;
        control_torque[i] = squeeze->control_torque[i] + (maintain_active ? maintain->control_torque[i] : 0);
    }
}

void PrimitiveController::setControlState(eControllerState state) {
    if (state == PRESHAPE && (cntrl_state == NONE || cntrl_state == MAINTAIN)) {
        std::cerr << "[" << primitive->name << "]: PRESHAPE" << std::endl;
        cntrl_state = PRESHAPE;
    }

    if (state == CLOSE && cntrl_state == PRESHAPE) {
        std::cerr << "[" << primitive->name << "]: CLOSE" << std::endl;
        squeeze->setTargetForClose();
        contact.init(contact_window_size, contact_interval);
        cntrl_state = CLOSE;
    }

    if (state == MAINTAIN && cntrl_state == CLOSE) {
        std::cerr << "[" << primitive->name << "]: MAINTAIN" << std::endl;
        maintain->setTarget();
        squeeze->setTargetForSqueeze();
        cntrl_state = MAINTAIN;
    }
}

///////////////////////////////
/// ControlOnePatch
///////////////////////////////
ControlOnePatch::ControlOnePatch(Patch *p) {
    if (p->kdlchain.getNrOfJoints() > 0)
        ictrl.InitChain(p->kdlchain);
    control_torque.resize(p->kdlchain.getNrOfJoints());
    err.resize(6);
    patch = p;
}

void ControlOnePatch::setStiffnessPosition(double kp) {
    ictrl.SetStiffnessPosition_e(kp);
}

void ControlOnePatch::setStiffnessPosition(Eigen::Matrix3d& kp) {
    ictrl.SetStiffnessPosition_e(kp);
}

void ControlOnePatch::setDampingPosition(double kd) {
    ictrl.SetDampingPosition_e(kd);
}

void ControlOnePatch::setDampingPosition(Eigen::Matrix3d& kd) {
    ictrl.SetDampingPosition_e(kd);
}

void ControlOnePatch::setStiffnessOrientation(double kp) {
    ictrl.SetStiffnessOrientation_e(kp);
}

void ControlOnePatch::setDampingOrientation(double kd) {
    ictrl.SetDampingOrientation_e(kd);
}

///////////////////////////////
/// ControlPrimitiveBase
///////////////////////////////
ControlPrimitiveBase::ControlPrimitiveBase(Executive *ce, Primitive *primitive) {
    this->ce = ce;
    this->primitive = primitive;
    if (primitive != NULL) {
        initPatchControl(cntrl_vf0, primitive->vf0);
        initPatchControl(cntrl_vf1, primitive->vf1);
    }
}

void ControlPrimitiveBase::initPatchControl(std::vector<ControlOnePatch>& patch_cntrl, std::vector<Patch>& vf) {
    for (std::vector<Patch>::iterator it = vf.begin(); it != vf.end(); ++it) {
        ControlOnePatch pic = ControlOnePatch(&(*it));
        std::map<std::string, PatchInfo>::iterator pit = patchInfo.find(pic.patch->name);
        if (pit == patchInfo.end()) {
            std::cerr << "No patch in the map" << std::endl;
            assert(0);
        }
        PatchInfo& pinfo = (*pit).second;

        pic.setStiffnessPosition(pinfo.kp_pos);
        pic.setDampingPosition(pinfo.kd_pos);
        pic.setStiffnessOrientation(pinfo.kp_rot);
        pic.setDampingOrientation(pinfo.kd_rot);

        for (int i=0; i<pinfo.joint_effort_mask.size(); i++)
            pic.ictrl.SetJointEffortMask(i, pinfo.joint_effort_mask[i]);

        patch_cntrl.push_back(pic);
    }
}


int main(int argc, char **argv) {
  initPatchInfo();
  ros::init(argc, argv, "allegro_primitive_control");
  Executive pc;
  ros::Duration(1).sleep();
  pc.initController();

  while (ros::ok()) {
    pc.updateController();
    ros::spinOnce();
  }
}
