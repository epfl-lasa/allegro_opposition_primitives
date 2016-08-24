#include "executive.h"

#include <string>

GravCompController::GravCompController(Executive *ce, Primitive *primitive)
    : ControlPrimitiveBase(ce, primitive) {
    control_torque.resize(DOF_JOINTS);

//    KDL::Vector kdlvec_gravity = KDL::Vector(0.0, 0.0, -25.8);
//    std::cout << "adding grav comp chain" << std::endl;
//    addChain("thumb_tip", kdlvec_gravity);
//    addChain("index_tip", kdlvec_gravity);
//    addChain("middle_tip", kdlvec_gravity);
//    addChain("pinky_tip", kdlvec_gravity);
}

void GravCompController::addChain(std::string name, KDL::Vector& kdlvec_gravity) {
    int i;

    std::string pname = patchNameMap[name];
    tips.push_back(Patch(pname, 1, ce->getTree())); i = tips.size()-1;
    gtorque.push_back(KDL::JntArray());
    gtorque[i].resize(tips[i].kdlchain.getNrOfJoints());
    gDynParams.push_back(new KDL::ChainDynParam(tips[i].kdlchain, kdlvec_gravity));
}

// the hack routine
void GravCompController::setTarget() {
    for (int i=0; i<DOF_JOINTS; i++)
        control_torque[i] = ce->current_joint_state.effort[i];

//-0.00133913 -0.14274 -0.0665326 -0.0105999 -0.0210184 -0.104237 -0.0843553 0.000990899 0.00999007 -0.207476 -0.0325397 -0.00418564 0.0266755 0.00435154 0.178603 0.100787
}

//void GravCompController::setTarget() {
//    for (int i=0; i < control_torque.size(); i++)
//        control_torque[i] = 0;

//    for (int i=0; i < tips.size(); i++) {
//        // update each patch
//        tips[i].updateState(ce->current_joint_state);
//        gDynParams[i]->JntToGravity(tips[i].kdl_joint_pos_cur, gtorque[i]);

//        std::map<std::string, PatchInfo>::iterator pit = patchInfo.find(tips[i].name);
//        if (pit == patchInfo.end()) {
//            std::cerr << "No patch in the map" << std::endl;
//            assert(0);
//        }
//        PatchInfo& pinfo = (*pit).second;

//        for (int j=0; j < tips[i].njoints; j++)
//            control_torque[pinfo.joint_map[j]] = (gtorque[i])(j);
//    }
//}

void GravCompController::setGravityVector(KDL::Vector &gvec) {
    for (int i=0; i < tips.size(); i++) {
        delete(gDynParams[i]);
        gDynParams[i] = new KDL::ChainDynParam(tips[i].kdlchain, gvec);
    }
}
