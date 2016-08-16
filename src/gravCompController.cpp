#include "executive.h"

GravCompController::GravCompController(Executive *ce, Primitive *primitive)
    : ControlPrimitiveBase(ce, primitive) {
    control_torque.resize(DOF_JOINTS);
}

void GravCompController::setTarget() {
    for (int i=0; i<DOF_JOINTS; i++)
        control_torque[i] = ce->current_joint_state.effort[i];
}

