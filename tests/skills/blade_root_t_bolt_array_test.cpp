// @lat: [[process/test-strategy#wind blade_root_t_bolt_array]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blade_root_t_bolt_array.hpp"

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

skill::blade_root_t_bolt_array::Input goodInput(const skill::Workpiece& wp) {
    skill::blade_root_t_bolt_array::Input in;
    in.face_id          = findTopFaceId(wp);
    in.center_x_mm      = 300.0;
    in.center_y_mm      = 300.0;
    in.bolt_pcd_mm      = 480.0;
    in.bolt_count       = 12;   // small but > 4 (full utility = 60-120, small for test perf)
    in.t_bolt_dia_mm    = 30.0; // M30
    in.t_bolt_depth_mm  = 60.0;
    in.recess_dia_mm    = 50.0;
    in.recess_depth_mm  = 20.0;
    return in;
}
}  // namespace

// ─── 1. Apply removes material with 2*bolt_count subfeatures ─────────────
TEST(SkillBladeRootTBoltArray, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 80.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::blade_root_t_bolt_array::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + in.bolt_count);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              2 * in.bolt_count);
}

// ─── 2. Validate rejects bolt_count <= 4 ─────────────────────────────────
TEST(SkillBladeRootTBoltArray, ValidateRejectsTooFewBolts)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 80.0);
    auto in    = goodInput(*stock);
    in.bolt_count = 4;  // boundary; must be > 4 per DFM rule

    auto r = skill::blade_root_t_bolt_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLT-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::blade_root_t_bolt_array::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects recess_dia <= t_bolt_dia + 4 ───────────────────
TEST(SkillBladeRootTBoltArray, ValidateRejectsSmallRecess)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 80.0);
    auto in    = goodInput(*stock);
    in.recess_dia_mm = in.t_bolt_dia_mm + 2.0;  // < +4 required

    auto r = skill::blade_root_t_bolt_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RECESS") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records wind_feature_type + bolt_count ────────────────
TEST(SkillBladeRootTBoltArray, SignatureRecordsCompound)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 80.0);
    auto in    = goodInput(*stock);
    auto out   = skill::blade_root_t_bolt_array::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("blade_root_t_bolt_array"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("wind_feature_type").get<std::string>(),
              std::string("blade_root_t_bolt_array"));
    EXPECT_EQ(out.signature.pattern.at("bolt_count").get<int>(), in.bolt_count);
    EXPECT_NEAR(out.signature.pattern.at("bolt_pcd_mm").get<double>(),
                in.bolt_pcd_mm, 1e-6);
    EXPECT_GT(out.signature.pattern.at("derived_volume_removed_mm3")
                  .get<double>(), 0.0);
}

// ─── 5. Recognize via metadata replay ───────────────────────────────────
TEST(SkillBladeRootTBoltArray, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 80.0);
    auto in    = goodInput(*stock);
    auto out   = skill::blade_root_t_bolt_array::apply(*stock, in);

    auto cands = skill::blade_root_t_bolt_array::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("bolt_count").get<int>(),
              in.bolt_count);
    EXPECT_NEAR(cands[0].recovered_params.at("t_bolt_dia_mm").get<double>(),
                in.t_bolt_dia_mm, 1e-6);
}
