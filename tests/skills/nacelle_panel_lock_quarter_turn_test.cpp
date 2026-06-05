// @lat: [[process/test-strategy#wind nacelle_panel_lock_quarter_turn]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/nacelle_panel_lock_quarter_turn.hpp"

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

skill::nacelle_panel_lock_quarter_turn::Input goodInput(const skill::Workpiece& wp) {
    skill::nacelle_panel_lock_quarter_turn::Input in;
    in.face_id                 = findTopFaceId(wp);
    in.center_x_mm             = 30.0;
    in.center_y_mm             = 30.0;
    in.latch_body_dia_mm       = 16.0;
    in.latch_depth_mm          = 10.0;
    in.cam_engagement_depth_mm = 4.0;
    in.sealing_o_ring_size_key = "-116";   // AS568 ID=20.30 CS=2.62 → fits >16
    return in;
}
}  // namespace

// ─── 1. Apply produces 4 sub-features (compound through+pocket+cam+groove) ─
TEST(SkillNacellePanelLock, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 16.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::nacelle_panel_lock_quarter_turn::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 4);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects unknown AS568 key ──────────────────────────────
TEST(SkillNacellePanelLock, ValidateRejectsUnknownAS568)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 16.0);
    auto in    = goodInput(*stock);
    in.sealing_o_ring_size_key = "-999";   // not in _as568_table

    auto r = skill::nacelle_panel_lock_quarter_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AS568") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::nacelle_panel_lock_quarter_turn::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects cam_engagement_depth >= latch_depth ────────────
TEST(SkillNacellePanelLock, ValidateRejectsCamTooDeep)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 16.0);
    auto in    = goodInput(*stock);
    in.cam_engagement_depth_mm = in.latch_depth_mm;  // equal -> error

    auto r = skill::nacelle_panel_lock_quarter_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAM-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature is_compound + wind_feature_type ───────────────────────
TEST(SkillNacellePanelLock, SignatureRecordsCompound)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 16.0);
    auto in    = goodInput(*stock);
    auto out   = skill::nacelle_panel_lock_quarter_turn::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("nacelle_panel_lock_quarter_turn"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("wind_feature_type").get<std::string>(),
              std::string("nacelle_panel_lock_quarter_turn"));
    EXPECT_NEAR(
        out.signature.pattern.at("latch_body_dia_mm").get<double>(),
        in.latch_body_dia_mm, 1e-6);
    EXPECT_EQ(
        out.signature.pattern.at("sealing_o_ring_size_key").get<std::string>(),
        in.sealing_o_ring_size_key);
    EXPECT_GT(out.signature.pattern.at("derived_volume_removed_mm3")
                  .get<double>(), 0.0);
}

// ─── 5. Recognize via metadata replay ───────────────────────────────────
TEST(SkillNacellePanelLock, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 16.0);
    auto in    = goodInput(*stock);
    auto out   = skill::nacelle_panel_lock_quarter_turn::apply(*stock, in);

    auto cands = skill::nacelle_panel_lock_quarter_turn::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("sealing_o_ring_size_key")
                  .get<std::string>(), in.sealing_o_ring_size_key);
}
