#include "patchInfo.h"
#include <Eigen/Eigen>

std::map<std::string, PatchInfo> patchInfo;
std::map<std::string, std::string> patchNameMap = {
    {"thumb_tip", "link_15.0_tip"},
    {"thumb_dist", "link_15.0"},
    {"thumb_mid", "link_14.0"},

    {"index_tip", "link_3.0_tip"},
    {"index_tip_side", "link_3.0_tip_side"},
    {"index_dist", "link_3.0"},
    {"index_dist_side", "link_3.0_side"},
    {"index_mid", "link_2.0"},
    {"index_mid_side", "link_2.0_side"},
    {"index_prox", "link_1.0"},
    {"index_prox_side", "link_1.0_side"},

    {"middle_tip", "link_7.0_tip"},
    {"middle_tip_side", "link_7.0_tip_side"},
    {"middle_dist", "link_7.0"},
    {"middle_dist_side", "link_7.0_side"},
    {"middle_mid", "link_6.0"},
    {"middle_mid_side", "link_6.0_side"},
    {"middle_prox", "link_5.0"},
    {"middle_prox_side", "link_5.0_side"},

    {"pinky_tip", "link_11.0_tip"},
    {"pinky_tip_side", "link_11.0_tip_side"},
    {"pinky_dist", "link_11.0"},
    {"pinky_dist_side", "link_11.0_side"},
    {"pinky_mid", "link_10.0"},
    {"pinky_mid_side", "link_10.0_side"},
    {"pinky_prox", "link_9.0"},
    {"pinky_prox_side", "link_9.0_side"},

    {"palm_idx_dgt",  "palm_link_i.dist"},
    {"palm_idx_wrst", "palm_link_i.prox"},
    {"palm_mid_dgt",  "palm_link_m.dist"},
    {"palm_mid_wrst", "palm_link_m.prox"},
    {"palm_pky_dgt",  "palm_link_r.dist"},
    {"palm_pky_wrst", "palm_link_r.prox"}
};

PatchInfo::PatchInfo() {
    name = "";
//    kp_pos = 50; kd_pos = 20;
    kp_pos = 50; kd_pos = 20;
//    kp_pos = 0;  kd_pos = 0;
//  kp_rot = 1;  kd_rot = 0.4;
    kp_rot = 0.2;  kd_rot = 0.05;
//    kp_rot = 0;  kd_rot = 0;
}

void PatchInfo::copy(const PatchInfo& pinfo) {
    name = pinfo.name;
    for (int i=0; i<pinfo.joint_map.size(); i++)
        joint_map.push_back(pinfo.joint_map[i]);
    kp_pos = pinfo.kp_pos; kd_pos = pinfo.kd_pos;
    kp_rot = pinfo.kp_rot; kd_rot = pinfo.kd_rot;
    for (int i=0; i<pinfo.patch_extension.size(); i++)
        patch_extension.push_back(KDL::Segment(pinfo.patch_extension[i]));
    for (int i=0; i<pinfo.joint_effort_mask.size(); i++)
        joint_effort_mask.push_back(pinfo.joint_effort_mask[i]);
}

void initPatchInfo() {
    Eigen::Matrix<double, 6, 6> t_on_r_on = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix<double, 6, 6> t_on_r_off = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix<double, 6, 6> t_off_r_on = Eigen::MatrixXd::Identity(6,6);
    t_on_r_off.block<3,3>(3,3) = Eigen::Matrix3d::Zero();
    t_off_r_on.block<3,3>(0,0) = Eigen::Matrix3d::Zero();

    // THUMB
    PatchInfo& tt = patchInfo["link_15.0_tip"];
    tt.name = "link_15.0_tip";
    tt.joint_map.push_back(eJOINTNAME_THUMB_0); tt.joint_effort_mask.push_back(t_on_r_on);
    tt.joint_map.push_back(eJOINTNAME_THUMB_1); tt.joint_effort_mask.push_back(t_on_r_off);
    tt.joint_map.push_back(eJOINTNAME_THUMB_2); tt.joint_effort_mask.push_back(t_on_r_off);
    tt.joint_map.push_back(eJOINTNAME_THUMB_3); tt.joint_effort_mask.push_back(t_on_r_off);

    PatchInfo& t3 = patchInfo["link_15.0"];
    t3.name = "link_15.0";
    t3.joint_map.push_back(eJOINTNAME_THUMB_0); t3.joint_effort_mask.push_back(t_on_r_on);
    t3.joint_map.push_back(eJOINTNAME_THUMB_1); t3.joint_effort_mask.push_back(t_on_r_off);
    t3.joint_map.push_back(eJOINTNAME_THUMB_2); t3.joint_effort_mask.push_back(t_on_r_off);
    t3.joint_map.push_back(eJOINTNAME_THUMB_3); t3.joint_effort_mask.push_back(t_on_r_off);
    t3.patch_extension.push_back(KDL::Segment("15.0_15.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.003) )));

    PatchInfo& t2 = patchInfo["link_14.0"];
    t2.name = "link_14.0";
    t2.joint_map.push_back(eJOINTNAME_THUMB_0); t2.joint_effort_mask.push_back(t_on_r_on);
    t2.joint_map.push_back(eJOINTNAME_THUMB_1); t2.joint_effort_mask.push_back(t_on_r_off);
    t2.joint_map.push_back(eJOINTNAME_THUMB_2); t2.joint_effort_mask.push_back(t_on_r_off);
    t2.patch_extension.push_back(KDL::Segment("14.0_14.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.018) )));


    // INDEX
    PatchInfo& it = patchInfo["link_3.0_tip"];
    it.name = "link_3.0_tip";
    it.joint_map.push_back(eJOINTNAME_INDEX_0); it.joint_effort_mask.push_back(t_on_r_off);
    it.joint_map.push_back(eJOINTNAME_INDEX_1); it.joint_effort_mask.push_back(t_on_r_off);
    it.joint_map.push_back(eJOINTNAME_INDEX_2); it.joint_effort_mask.push_back(t_on_r_off);
    it.joint_map.push_back(eJOINTNAME_INDEX_3); it.joint_effort_mask.push_back(t_on_r_off);
    PatchInfo& it_side = patchInfo["link_3.0_tip_side"];
    it_side.copy(it);
    it_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& i3 = patchInfo["link_3.0"];
//    i3.name = "link_2.0";
    i3.name = "link_3.0";
    i3.joint_map.push_back(eJOINTNAME_INDEX_0); i3.joint_effort_mask.push_back(t_on_r_off);
    i3.joint_map.push_back(eJOINTNAME_INDEX_1); i3.joint_effort_mask.push_back(t_on_r_off);
    i3.joint_map.push_back(eJOINTNAME_INDEX_2); i3.joint_effort_mask.push_back(t_on_r_off);
    i3.joint_map.push_back(eJOINTNAME_INDEX_3); i3.joint_effort_mask.push_back(t_on_r_off);
//    i3.patch_extension.push_back(KDL::Segment("2.0_3.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.03828) )));
    i3.patch_extension.push_back(KDL::Segment("3.0_3.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.014) )));
    PatchInfo& i3_side = patchInfo["link_3.0_side"];
    i3_side.copy(i3);
    i3_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& i2 = patchInfo["link_2.0"];
//    i2.name = "link_1.0";
    i2.name = "link_2.0";
    i2.joint_map.push_back(eJOINTNAME_INDEX_0); i2.joint_effort_mask.push_back(t_on_r_off);
    i2.joint_map.push_back(eJOINTNAME_INDEX_1); i2.joint_effort_mask.push_back(t_on_r_off);
    i2.joint_map.push_back(eJOINTNAME_INDEX_2); i2.joint_effort_mask.push_back(t_on_r_off);
//    i2.patch_extension.push_back(KDL::Segment("1.0_2.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.05371) )));
    i2.patch_extension.push_back(KDL::Segment("2.0_2.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.015) )));
    PatchInfo& i2_side = patchInfo["link_2.0_side"];
    i2_side.copy(i2);
    i2_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& i1 = patchInfo["link_1.0"];
    i1.name = "link_1.0";
    i1.joint_map.push_back(eJOINTNAME_INDEX_0); i1.joint_effort_mask.push_back(t_on_r_off);
    i1.joint_map.push_back(eJOINTNAME_INDEX_1); i1.joint_effort_mask.push_back(t_on_r_off);
    i1.patch_extension.push_back(KDL::Segment("1.0_1.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.015) )));
    PatchInfo& i1_side = patchInfo["link_1.0_side"];
    i1_side.copy(i1);
    i1_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    // MIDDLE
    PatchInfo& mt = patchInfo["link_7.0_tip"];
    mt.name = "link_7.0_tip";
    mt.joint_map.push_back(eJOINTNAME_MIDDLE_0); mt.joint_effort_mask.push_back(t_on_r_off);
    mt.joint_map.push_back(eJOINTNAME_MIDDLE_1); mt.joint_effort_mask.push_back(t_on_r_off);
    mt.joint_map.push_back(eJOINTNAME_MIDDLE_2); mt.joint_effort_mask.push_back(t_on_r_off);
    mt.joint_map.push_back(eJOINTNAME_MIDDLE_3); mt.joint_effort_mask.push_back(t_on_r_off);
    PatchInfo& mt_side = patchInfo["link_7.0_tip_side"];
    mt_side.copy(mt);
    mt_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& m3 = patchInfo["link_7.0"];
    m3.name = "link_7.0";
    m3.joint_map.push_back(eJOINTNAME_MIDDLE_0); m3.joint_effort_mask.push_back(t_on_r_off);
    m3.joint_map.push_back(eJOINTNAME_MIDDLE_1); m3.joint_effort_mask.push_back(t_on_r_off);
    m3.joint_map.push_back(eJOINTNAME_MIDDLE_2); m3.joint_effort_mask.push_back(t_on_r_off);
    m3.joint_map.push_back(eJOINTNAME_MIDDLE_3); m3.joint_effort_mask.push_back(t_on_r_off);
    m3.patch_extension.push_back(KDL::Segment("7.0_7.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.014) )));
    PatchInfo& m3_side = patchInfo["link_7.0_side"];
    m3_side.copy(m3);
    m3_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& m2 = patchInfo["link_6.0"];
    m2.name = "link_6.0";
    m2.joint_map.push_back(eJOINTNAME_MIDDLE_0); m2.joint_effort_mask.push_back(t_on_r_off);
    m2.joint_map.push_back(eJOINTNAME_MIDDLE_1); m2.joint_effort_mask.push_back(t_on_r_off);
    m2.joint_map.push_back(eJOINTNAME_MIDDLE_2); m2.joint_effort_mask.push_back(t_on_r_off);
    m2.patch_extension.push_back(KDL::Segment("6.0_6.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.015) )));
    PatchInfo& m2_side = patchInfo["link_6.0_side"];
    m2_side.copy(m2);
    m2_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& m1 = patchInfo["link_5.0"];
    m1.name = "link_5.0";
    m1.joint_map.push_back(eJOINTNAME_MIDDLE_0); m1.joint_effort_mask.push_back(t_on_r_off);
    m1.joint_map.push_back(eJOINTNAME_MIDDLE_1); m1.joint_effort_mask.push_back(t_on_r_off);
    m1.patch_extension.push_back(KDL::Segment("5.0_5.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.015) )));
    PatchInfo& m1_side = patchInfo["link_5.0_side"];
    m1_side.copy(m1);
    m1_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    // PINKY
    PatchInfo& pt = patchInfo["link_11.0_tip"];
    pt.name = "link_11.0_tip";
    pt.joint_map.push_back(eJOINTNAME_PINKY_0); pt.joint_effort_mask.push_back(t_on_r_off);
    pt.joint_map.push_back(eJOINTNAME_PINKY_1); pt.joint_effort_mask.push_back(t_on_r_off);
    pt.joint_map.push_back(eJOINTNAME_PINKY_2); pt.joint_effort_mask.push_back(t_on_r_off);
    pt.joint_map.push_back(eJOINTNAME_PINKY_3); pt.joint_effort_mask.push_back(t_on_r_off);
    PatchInfo& pt_side = patchInfo["link_11.0_tip_side"];
    pt_side.copy(pt);
    pt_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& p3 = patchInfo["link_11.0"];
    p3.name = "link_11.0";
    p3.joint_map.push_back(eJOINTNAME_PINKY_0); p3.joint_effort_mask.push_back(t_on_r_off);
    p3.joint_map.push_back(eJOINTNAME_PINKY_1); p3.joint_effort_mask.push_back(t_on_r_off);
    p3.joint_map.push_back(eJOINTNAME_PINKY_2); p3.joint_effort_mask.push_back(t_on_r_off);
    p3.joint_map.push_back(eJOINTNAME_PINKY_3); p3.joint_effort_mask.push_back(t_on_r_off);
    p3.patch_extension.push_back(KDL::Segment("11.0_11.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.014) )));
    PatchInfo& p3_side = patchInfo["link_11.0_side"];
    p3_side.copy(p3);
    p3_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& p2 = patchInfo["link_10.0"];
    p2.name = "link_10.0";
    p2.joint_map.push_back(eJOINTNAME_PINKY_0); p2.joint_effort_mask.push_back(t_on_r_off);
    p2.joint_map.push_back(eJOINTNAME_PINKY_1); p2.joint_effort_mask.push_back(t_on_r_off);
    p2.joint_map.push_back(eJOINTNAME_PINKY_2); p2.joint_effort_mask.push_back(t_on_r_off);
    p2.patch_extension.push_back(KDL::Segment("10.0_10.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.015) )));
    PatchInfo& p2_side = patchInfo["link_10.0_side"];
    p2_side.copy(p2);
    p2_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    PatchInfo& p1 = patchInfo["link_9.0"];
    p1.name = "link_9.0";
    p1.joint_map.push_back(eJOINTNAME_PINKY_0); p1.joint_effort_mask.push_back(t_on_r_off);
    p1.joint_map.push_back(eJOINTNAME_PINKY_1); p1.joint_effort_mask.push_back(t_on_r_off);
    p1.patch_extension.push_back(KDL::Segment("9.0_9.0", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.015) )));
    PatchInfo& p1_side = patchInfo["link_9.0_side"];
    p1_side.copy(p1);
    p1_side.patch_extension.push_back(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::RotZ(KDL::deg2rad*90.0), KDL::Vector(0.0, 0.0, 0.0) )));


    // PALM
    PatchInfo& pproxi = patchInfo["palm_link_i.prox"];
    pproxi.name = "palm_link";
    pproxi.patch_extension.push_back(KDL::Segment("palm_link_i.prox", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.013, 0.045, -0.023) )));

    PatchInfo& pdisti = patchInfo["palm_link_i.dist"];
    pdisti.name = "palm_link";
    pdisti.patch_extension.push_back(KDL::Segment("palm_link_i.dist", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.013, 0.045, -0.045) )));

    PatchInfo& pproxm = patchInfo["palm_link_m.prox"];
    pproxm.name = "palm_link";
    pproxm.patch_extension.push_back(KDL::Segment("palm_link_m.prox", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.013, 0.0, -0.023) )));

    PatchInfo& pdistm = patchInfo["palm_link_m.dist"];
    pdistm.name = "palm_link";
    pdistm.patch_extension.push_back(KDL::Segment("palm_link_m.dist", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.013, 0.0, -0.045) )));

    PatchInfo& pproxr = patchInfo["palm_link_r.prox"];
    pproxr.name = "palm_link";
    pproxr.patch_extension.push_back(KDL::Segment("palm_link_r.prox", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.013, -0.045, -0.023) )));

    PatchInfo& pdistr = patchInfo["palm_link_r.dist"];
    pdistr.name = "palm_link";
    pdistr.patch_extension.push_back(KDL::Segment("palm_link_r.dist", KDL::Joint(KDL::Joint::None), KDL::Frame( KDL::Rotation::Identity(), KDL::Vector(0.013, -0.045, -0.045) )));


//    for (std::map<std::string, PatchInfo>::iterator it = patchInfo.begin(); it != patchInfo.end(); ++it)
//        std::cerr << (*it).second.name << std::endl;
}
