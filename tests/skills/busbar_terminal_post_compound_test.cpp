// @lat: [[process/test-strategy#compound busbar_terminal_post_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/busbar_terminal_post_compound.hpp"

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

skill::busbar_terminal_post_compound::Input goodInput(const skill::Workpiece& wp) {
    skill::busbar_terminal_post_compound::Input in;
    in.face_id                   = findTopFaceId(wp);
    in.center_x_mm               = 25.0;
    in.center_y_mm               = 25.0;
    in.post_dia_mm               = 10.0;
    in.post_height_mm            = 12.0;
    in.thread_size_key           = "M6";
    in.insulator_recess_dia_mm   = 14.0;
    in.insulator_recess_depth_mm = 1.0;
    return in;
}
}  // namespace

// ─── 1. Apply: net volume changes (post added > pilot+recess removed) ────
TEST(SkillBusbarPost, ApplyFusesPostAndCutsPilotAndRecess)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::busbar_terminal_post_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    // Net change should be positive (post added > pilot+recess removed).
    EXPECT_GT(v1, v0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects unknown thread key ──────────────────────────────
TEST(SkillBusbarPost, ValidateRejectsUnknownThreadKey)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    in.thread_size_key = "M99";  // not in central table

    auto r = skill::busbar_terminal_post_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD-KEY") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::busbar_terminal_post_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects insufficient insulator gap ──────────────────────
TEST(SkillBusbarPost, ValidateRejectsInsulatorGapTooSmall)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    in.insulator_recess_dia_mm = in.post_dia_mm + 0.2;  // < 0.5 gap

    auto r = skill::busbar_terminal_post_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INSULATOR-GAP") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + EV feature type ─────────────────────
TEST(SkillBusbarPost, SignatureRecordsCompoundEvFeature)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    auto out   = skill::busbar_terminal_post_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("busbar_terminal_post_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("ev_feature_type").get<std::string>(),
              std::string("busbar_terminal_post"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_EQ(out.signature.pattern.at("thread_size_key").get<std::string>(),
              std::string("M6"));
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillBusbarPost, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in    = goodInput(*stock);
    auto out   = skill::busbar_terminal_post_compound::apply(*stock, in);

    auto cands = skill::busbar_terminal_post_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("thread_size_key").get<std::string>(),
              std::string("M6"));
}
