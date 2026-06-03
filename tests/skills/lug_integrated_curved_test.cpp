// @lat: [[process/test-strategy#compound lug_integrated_curved]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lug_integrated_curved.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::lug_integrated_curved::Input goodInput() {
    skill::lug_integrated_curved::Input in;
    in.case_side_face         = skill::FaceByNormal{ gp_Dir(1, 0, 0) };
    in.case_axis              = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    in.case_outer_radius_mm   = 20.0;
    in.lug_count              = 4;
    in.lug_angle_deg          = 80.0;
    in.lug_width_mm           = 4.0;
    in.lug_thickness_mm       = 2.5;
    in.lug_length_mm          = 7.0;
    in.spring_bar_dia_mm      = 1.5;
    in.spring_bar_z_offset_mm = 0.0;
    return in;
}
}  // namespace

// ─── 1. Apply adds 4 lug volumes minus 4 spring-bar holes ────────────────
TEST(SkillLugIntegratedCurved, ApplyAddsLugsAndDrillsBars)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::lug_integrated_curved::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double perLug = in.lug_width_mm * in.lug_thickness_mm * in.lug_length_mm;
    const double addedExpected = perLug * 4.0;
    const double v1 = volumeOf(stock->shape());
    const double v2 = volumeOf(out.workpiece->shape());
    const double netAdded = v2 - v1;
    EXPECT_GT(netAdded, addedExpected * 0.30);
    EXPECT_LT(netAdded, addedExpected * 1.10);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects bad lug_count ───────────────────────────────────
TEST(SkillLugIntegratedCurved, ValidateRejectsBadLugCount)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.lug_count = 5;   // not in {4, 6}

    auto r = skill::lug_integrated_curved::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LUG-COUNT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::lug_integrated_curved::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects non-catalog spring bar ──────────────────────────
TEST(SkillLugIntegratedCurved, ValidateRejectsBadSpringBar)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    in.spring_bar_dia_mm = 1.0;   // not catalog

    auto r = skill::lug_integrated_curved::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SPRINGBAR-SIZE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + integrated lugs + count ─────────────
TEST(SkillLugIntegratedCurved, SignatureRecordsIntegratedLugs)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::lug_integrated_curved::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("lug_integrated_curved"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_watch_feature").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("watch_feature_type").get<std::string>(),
              std::string("integrated_curved_lugs"));
    EXPECT_EQ(out.signature.pattern.at("lug_count").get<int>(), 4);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 8);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillLugIntegratedCurved, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 8.0);

    auto in = goodInput();
    auto out = skill::lug_integrated_curved::apply(*stock, in);

    auto cands = skill::lug_integrated_curved::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("lug_count").get<int>(), 4);
}
