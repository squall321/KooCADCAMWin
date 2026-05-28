// @lat: [[process/test-strategy#skill round-trip]]
//
// gear_tooth_cut — 5 tests covering geometry / DFM / signature / recognize.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gear_tooth_cut.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

// ─── 1. Geometry: volume decreases by ~ wedge box volume ──────────────────
TEST(SkillGearToothCut, ApplyChangesGeometry)
{
    auto stock = skill::createCylindricalStock(22.0, 5.0);   // dia 22 → R=11
    ASSERT_FALSE(stock->shape().IsNull());

    skill::gear_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.angle_rad        = 0.0;
    in.module_mm        = 1.0;
    in.pressure_angle_deg = 20.0;
    in.num_teeth        = 20;
    in.root_radius_mm   = 9.0;
    in.tip_radius_mm    = 11.0;
    in.face_width_mm    = 5.0;

    const double v0 = volumeOf(stock->shape());
    auto out = skill::gear_tooth_cut::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0) << "wedge cut must remove material";

    // Approximate wedge box volume = tangent * radial * axial.
    const double approxRemoved =
        (M_PI * in.module_mm / 2.0) *               // tangent (full pitch/2)
        (in.tip_radius_mm - in.root_radius_mm) *    // radial
        in.face_width_mm;                            // axial
    const double removed = v0 - v1;
    EXPECT_NEAR(removed, approxRemoved, approxRemoved * 0.5)
        << "wedge approximation removed volume in the expected ballpark";
}

// ─── 2. DFM rejects bad module ────────────────────────────────────────────
TEST(SkillGearToothCut, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(22.0, 5.0);

    skill::gear_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.module_mm        = 0.05;     // out of range: < 0.2
    in.pressure_angle_deg = 20.0;
    in.root_radius_mm   = 9.0;
    in.tip_radius_mm    = 11.0;
    in.face_width_mm    = 5.0;

    auto r = skill::gear_tooth_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GEAR-MOD") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::gear_tooth_cut::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + gear params ──────────────────────────────
TEST(SkillGearToothCut, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(22.0, 5.0);

    skill::gear_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.angle_rad        = M_PI / 4.0;
    in.module_mm        = 1.5;
    in.pressure_angle_deg = 14.5;
    in.helix_angle_deg  = 0.0;
    in.num_teeth        = 24;
    in.root_radius_mm   = 8.0;
    in.tip_radius_mm    = 11.0;
    in.face_width_mm    = 4.0;

    auto out = skill::gear_tooth_cut::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("gear_tooth_cut"));
    EXPECT_EQ(out.signature.pattern["kind"], "gear_tooth_cut");
    EXPECT_NEAR(out.signature.params["module_mm"].get<double>(), 1.5, 1e-9);
    EXPECT_NEAR(out.signature.params["pressure_angle_deg"].get<double>(), 14.5, 1e-9);
    EXPECT_EQ(out.signature.params["num_teeth"].get<int>(), 24);
    EXPECT_NEAR(out.signature.pattern["root_radius_mm"].get<double>(), 8.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["tip_radius_mm"].get<double>(), 11.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["approximation"], "wedge_box");
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillGearToothCut, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(22.0, 5.0);

    skill::gear_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.module_mm        = 1.0;
    in.pressure_angle_deg = 20.0;
    in.root_radius_mm   = 9.0;
    in.tip_radius_mm    = 11.0;
    in.face_width_mm    = 5.0;

    auto out = skill::gear_tooth_cut::apply(*stock, in);
    auto cands = skill::gear_tooth_cut::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("gear_tooth_cut"));
    EXPECT_GT(cands[0].confidence, 0.5);
    EXPECT_NEAR(cands[0].recovered_params["module_mm"].get<double>(),
                1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["tip_radius_mm"].get<double>(),
                11.0, 1e-9);
}

// ─── 5. Specific: tooling metadata includes hob tool_type ─────────────────
TEST(SkillGearToothCut, ToolingTypeIsGearHob)
{
    auto stock = skill::createCylindricalStock(22.0, 5.0);

    skill::gear_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.module_mm        = 1.0;
    in.pressure_angle_deg = 20.0;
    in.root_radius_mm   = 9.0;
    in.tip_radius_mm    = 11.0;
    in.face_width_mm    = 5.0;

    auto out = skill::gear_tooth_cut::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("gear_hob"));
    EXPECT_GT(out.signature.tooling.stock_removed_mm3, 0.0);
}
