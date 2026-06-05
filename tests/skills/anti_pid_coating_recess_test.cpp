// @lat: [[process/test-strategy#compound anti_pid_coating_recess]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/anti_pid_coating_recess.hpp"

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

skill::anti_pid_coating_recess::Input goodInput(const skill::Workpiece& wp) {
    skill::anti_pid_coating_recess::Input in;
    in.face_id            = findTopFaceId(wp);
    in.origin_x_mm        = 15.0;
    in.origin_y_mm        = 15.0;
    in.cell_array_count_x = 2;
    in.cell_array_count_y = 2;
    in.cell_pitch_x_mm    = 12.0;
    in.cell_pitch_y_mm    = 12.0;
    in.recess_depth_mm    = 0.5;
    in.coating_margin_mm  = 0.8;
    return in;
}
}  // namespace

// ─── 1. Apply: cuts N×M shallow cell recesses ────────────────────────────
TEST(SkillAntiPidRecess, ApplySequentialCutsRecessArray)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::anti_pid_coating_recess::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects deep recess (> anti-PID range) ──────────────────
TEST(SkillAntiPidRecess, ValidateRejectsDeepRecess)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    auto in    = goodInput(*stock);
    in.recess_depth_mm = 2.5;          // > 1.5 mm cap

    auto r = skill::anti_pid_coating_recess::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PID-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects oversize array ──────────────────────────────────
TEST(SkillAntiPidRecess, ValidateRejectsOversizeArray)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    auto in    = goodInput(*stock);
    in.cell_array_count_x = 30;         // > 24

    auto r = skill::anti_pid_coating_recess::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PID-ARRAY-SIZE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + cell count ──────────────────────────
TEST(SkillAntiPidRecess, SignatureRecordsCellCount)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    auto in    = goodInput(*stock);
    auto out   = skill::anti_pid_coating_recess::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("anti_pid_coating_recess"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("solar_feature_type").get<std::string>(),
              std::string("anti_pid_recess_array"));
    EXPECT_EQ(out.signature.pattern.at("cell_count").get<int>(),
              in.cell_array_count_x * in.cell_array_count_y);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              in.cell_array_count_x * in.cell_array_count_y);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillAntiPidRecess, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 5.0);
    auto in    = goodInput(*stock);
    auto out   = skill::anti_pid_coating_recess::apply(*stock, in);

    auto cands = skill::anti_pid_coating_recess::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("recess_depth_mm").get<double>(),
              in.recess_depth_mm);
}
