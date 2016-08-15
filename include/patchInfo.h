#ifndef PATCH_INFO_H
#define PATCH_INFO_H

#include <vector>
#include <map>
#include <string>
#include <allegro_hand_driver/controlAllegroHand.h>
#include <kdl/segment.hpp>
#include <Eigen/Eigen>

class PatchInfo {
public:
    PatchInfo();
    void copy(const PatchInfo& pinfo);

    std::string name;
    std::vector<eJointName> joint_map;
    double kp_pos, kd_pos, kp_rot, kd_rot;
    std::vector<KDL::Segment> patch_extension;
    std::vector<Eigen::Matrix<double, 6, 6> > joint_effort_mask;
};

extern std::map<std::string, PatchInfo> patchInfo;
extern std::map<std::string, std::string> patchNameMap;
void initPatchInfo();

#endif // PATCH_INFO_H
