// @lat: [[process/test-strategy#compound pcb_lock_tab_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pcb_lock_tab_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFaceId(const skill::Workpiece& wp) {
    int best = -1;
    double bestZ = -1e9;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const auto n = wp.faceNormal(i);
        if (n.Z() < 0.9) continue;
        const auto c = wp.faceCenter(i);
        if (c.Z() > bestZ) { bestZ = c.Z(); best = i; }
    }
    return best;
}

skill::pcb_lock_tab_compound::Input goodInput(const skill::Workpiece& wp) {
    skill::pcb_lock_tab_compound::Input in;
    in.face_id                  = findTopFaceId(wp);
    in.tab_position_x_mm        = 20.0;
    in.tab_position_y_mm        = 20.0;
    in.tab_width_mm             = 4.0;
    in.tab_length_mm            = 7.0;
    in.tab_height_above_face_mm = 1.5;
    in.slot_engagement_depth_mm = 0.8;
    return in;
}
}  // namespace

// ─── 1. Apply fuses tab and cuts engagement slot ─────────────────────────
TEST(SkillPcbLockTab, ApplyFusesTabAndCutsSlot)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::pcb_lock_tab_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    // Tab is added on top, slot is cut into it — net volume should be
    // positive (tab box > slot cut).
    EXPECT_GT(v1, v0);
}

// ─── 2. Validate rejects tab too short ──────────────────────────────────
TEST(SkillPcbLockTab, ValidateRejectsTabHeightTooSmall)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    auto in    = goodInput(*stock);
    in.tab_height_above_face_mm = 0.2;  // < 0.5

    auto r = skill::pcb_lock_tab_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TAB-HEIGHT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects slot deeper than tab height ─────────────────────
TEST(SkillPcbLockTab, ValidateRejectsSlotDeeperThanTab)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    auto in    = goodInput(*stock);
    in.slot_engagement_depth_mm = in.tab_height_above_face_mm + 0.2;

    auto r = skill::pcb_lock_tab_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT-VS-TAB") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + ev_feature_type ─────────────────────
TEST(SkillPcbLockTab, SignatureRecordsCompoundAndType)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    auto in    = goodInput(*stock);
    auto out   = skill::pcb_lock_tab_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("pcb_lock_tab_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("ev_feature_type").get<std::string>(),
              std::string("pcb_lock_tab"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillPcbLockTab, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    auto in    = goodInput(*stock);
    auto out   = skill::pcb_lock_tab_compound::apply(*stock, in);

    auto cands = skill::pcb_lock_tab_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("tab_width_mm").get<double>(),
              in.tab_width_mm);
}
