// @lat: [[process/test-strategy#skill compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pcb_card_edge_socket.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// 1. Apply removes real geometry: slot + N reliefs + key tab notch.
TEST(SkillPcbCardEdgeSocket, ApplyCutsRealGeometry)
{
    // Socket body: long bar.  Slot will be along +X.
    auto stock = skill::createCuboidStock(80.0, 12.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();

    skill::pcb_card_edge_socket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 5.0;
    in.start_y_mm        = 6.0;
    in.end_x_mm          = 65.0;
    in.end_y_mm          = 6.0;
    in.slot_width_mm     = 1.5;
    in.slot_depth_mm     = 5.0;
    in.finger_count      = 22;
    in.pitch_mm          = 2.54;
    in.finger_dia_mm     = 1.5;
    in.finger_depth_mm   = 4.5;
    in.key_tab_width_mm  = 2.0;
    in.key_tab_depth_mm  = 1.5;
    in.key_tab_offset_mm = 15.0;

    auto out = skill::pcb_card_edge_socket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double slotLen = 60.0;
    const double rF = 0.75;
    const double approxRem =
        slotLen * 1.5 * 5.0                                  // main slot
      + 22 * M_PI * rF * rF * 4.5                            // 22 finger reliefs
      + 2.0 * (1.5 + 2.0 * 1.5) * 5.0;                       // key tab notch
    const double diff = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approxRem, approxRem * 0.20);
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
}

// 2. Validate rejects bad input + apply throws.
TEST(SkillPcbCardEdgeSocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(80.0, 12.0, 10.0);
    skill::pcb_card_edge_socket::Input bad;
    bad.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.start_x_mm        = 5.0;
    bad.start_y_mm        = 6.0;
    bad.end_x_mm          = 65.0;
    bad.end_y_mm          = 6.0;
    bad.slot_width_mm     = 1.5;
    bad.slot_depth_mm     = 5.0;
    bad.finger_count      = 22;
    bad.pitch_mm          = 1.0;          // not in {1.27, 2.0, 2.54} — DFM-JEDEC-MO189
    bad.finger_dia_mm     = 0.7;
    bad.finger_depth_mm   = 4.5;
    bad.key_tab_width_mm  = 2.0;
    bad.key_tab_depth_mm  = 1.5;
    bad.key_tab_offset_mm = 15.0;

    auto r = skill::pcb_card_edge_socket::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-JEDEC-MO189") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::pcb_card_edge_socket::apply(*stock, bad), skill::SkillError);
}

// 3. Signature: compound, subfeature_count >= 2, params round-trip.
TEST(SkillPcbCardEdgeSocket, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(80.0, 12.0, 10.0);
    skill::pcb_card_edge_socket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 5.0;
    in.start_y_mm        = 6.0;
    in.end_x_mm          = 65.0;
    in.end_y_mm          = 6.0;
    in.slot_width_mm     = 1.5;
    in.slot_depth_mm     = 5.0;
    in.finger_count      = 22;
    in.pitch_mm          = 2.54;
    in.finger_dia_mm     = 1.5;
    in.finger_depth_mm   = 4.5;
    in.key_tab_width_mm  = 2.0;
    in.key_tab_depth_mm  = 1.5;
    in.key_tab_offset_mm = 15.0;

    auto out = skill::pcb_card_edge_socket::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_TRUE(pat["is_compound"].get<bool>());
    EXPECT_GE(pat["subfeature_count"].get<int>(), 2);
    EXPECT_EQ(pat["finger_count"].get<int>(), 22);

    const auto& p = out.signature.params;
    EXPECT_EQ(p["finger_count"].get<int>(), 22);
    EXPECT_DOUBLE_EQ(p["pitch_mm"].get<double>(), 2.54);
    EXPECT_DOUBLE_EQ(p["slot_width_mm"].get<double>(), 1.5);
}

// 4. Recognize: metadata replay 1.0; geometric ≥0.5.
TEST(SkillPcbCardEdgeSocket, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(80.0, 12.0, 10.0);
    skill::pcb_card_edge_socket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 5.0;
    in.start_y_mm        = 6.0;
    in.end_x_mm          = 65.0;
    in.end_y_mm          = 6.0;
    in.slot_width_mm     = 1.5;
    in.slot_depth_mm     = 5.0;
    in.finger_count      = 22;
    in.pitch_mm          = 2.54;
    in.finger_dia_mm     = 1.5;
    in.finger_depth_mm   = 4.5;
    in.key_tab_width_mm  = 2.0;
    in.key_tab_depth_mm  = 1.5;
    in.key_tab_offset_mm = 15.0;

    auto out = skill::pcb_card_edge_socket::apply(*stock, in);
    auto cands = skill::pcb_card_edge_socket::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool hasMeta = false;
    for (const auto& c : cands)
        if (std::abs(c.confidence - 1.0) < 1e-6) { hasMeta = true; break; }
    EXPECT_TRUE(hasMeta);
}

// 5. Feature-specific: finger_count round-trips through pattern.
TEST(SkillPcbCardEdgeSocket, FeatureSpecific_FingerCount)
{
    auto stock = skill::createCuboidStock(120.0, 12.0, 10.0);
    skill::pcb_card_edge_socket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm        = 5.0;
    in.start_y_mm        = 6.0;
    in.end_x_mm          = 105.0;
    in.end_y_mm          = 6.0;
    in.slot_width_mm     = 1.5;
    in.slot_depth_mm     = 5.0;
    in.finger_count      = 30;        // larger card
    in.pitch_mm          = 2.54;
    in.finger_dia_mm     = 1.5;
    in.finger_depth_mm   = 4.5;
    in.key_tab_width_mm  = 2.0;
    in.key_tab_depth_mm  = 1.5;
    in.key_tab_offset_mm = 20.0;

    auto out = skill::pcb_card_edge_socket::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["finger_count"].get<int>(), 30);
    EXPECT_EQ(out.signature.params["finger_count"].get<int>(),  30);
}
