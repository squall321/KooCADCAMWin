// @lat: [[process/test-strategy#medical cannulated screw lumen]]
//
// cannulated_screw_lumen — through lumen + hex head recess.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cannulated_screw_lumen.hpp"

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

// ─── 1. apply produces compound: lumen + head recess ─────────────────────
TEST(SkillCannulatedScrewLumen, ApplyProducesCompoundShape)
{
    // Cylindrical "screw body" stock: 8 mm dia × 30 mm length puck.
    auto stock = skill::createCylindricalStock(8.0, 30.0);
    const double stockVol = volumeOf(stock->shape());

    skill::cannulated_screw_lumen::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.screw_axis_dir    = gp_Dir(0, 0, -1);
    in.screw_axis_origin = gp_Pnt(0.0, 0.0, 30.0);    // top face of puck
    in.outer_dia_mm      = 8.0;
    in.lumen_dia_mm      = 2.0;       // < 0.5 × 8.0 = 4.0  ✓
    in.length_mm         = 30.0;
    in.head_dia_mm       = 10.0;      // wider than outer
    in.head_depth_mm     = 3.0;

    auto out = skill::cannulated_screw_lumen::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = stockVol - volumeOf(out.workpiece->shape());
    // Lumen volume alone ≈ π·(1.0)²·30 = 94.2 mm³. Material has been cut.
    EXPECT_GT(removed, 50.0);

    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("medical_feature_type").get<std::string>(),
              std::string("cannulated_bone_screw"));
    EXPECT_NEAR(out.signature.pattern.at("wall_thickness_mm").get<double>(),
                3.0, 1e-6);    // (8 - 2)/2
}

// ─── 2. validate rejects lumen ≥ 0.5 × outer ─────────────────────────────
TEST(SkillCannulatedScrewLumen, ValidateRejectsThickLumen)
{
    auto stock = skill::createCylindricalStock(8.0, 30.0);

    skill::cannulated_screw_lumen::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.screw_axis_dir    = gp_Dir(0, 0, -1);
    in.screw_axis_origin = gp_Pnt(0.0, 0.0, 30.0);
    in.outer_dia_mm      = 8.0;
    in.lumen_dia_mm      = 4.5;      // > 0.5 × outer → reject
    in.length_mm         = 30.0;
    in.head_dia_mm       = 10.0;
    in.head_depth_mm     = 3.0;

    auto r = skill::cannulated_screw_lumen::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CANN-LUMEN") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::cannulated_screw_lumen::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects head_dia ≤ outer_dia ────────────────────────────
TEST(SkillCannulatedScrewLumen, ValidateRejectsTinyHead)
{
    auto stock = skill::createCylindricalStock(8.0, 30.0);

    skill::cannulated_screw_lumen::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.screw_axis_dir    = gp_Dir(0, 0, -1);
    in.screw_axis_origin = gp_Pnt(0.0, 0.0, 30.0);
    in.outer_dia_mm      = 8.0;
    in.lumen_dia_mm      = 2.0;
    in.length_mm         = 30.0;
    in.head_dia_mm       = 7.5;      // ≤ outer → reject
    in.head_depth_mm     = 3.0;

    auto r = skill::cannulated_screw_lumen::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CANN-HEAD") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. recognize replays metadata ───────────────────────────────────────
TEST(SkillCannulatedScrewLumen, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(8.0, 30.0);

    skill::cannulated_screw_lumen::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.screw_axis_dir    = gp_Dir(0, 0, -1);
    in.screw_axis_origin = gp_Pnt(0.0, 0.0, 30.0);
    in.outer_dia_mm      = 8.0;
    in.lumen_dia_mm      = 1.6;
    in.length_mm         = 25.0;
    in.head_dia_mm       = 10.0;
    in.head_depth_mm     = 3.0;

    auto out = skill::cannulated_screw_lumen::apply(*stock, in);
    auto cands = skill::cannulated_screw_lumen::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("lumen_dia_mm").get<double>(),
                1.6, 1e-6);
}

// ─── 5. signature pattern marks lumen_through + records dims ─────────────
TEST(SkillCannulatedScrewLumen, SignatureRecordsKeyDimensions)
{
    auto stock = skill::createCylindricalStock(8.0, 30.0);

    skill::cannulated_screw_lumen::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.screw_axis_dir    = gp_Dir(0, 0, -1);
    in.screw_axis_origin = gp_Pnt(0.0, 0.0, 30.0);
    in.outer_dia_mm      = 8.0;
    in.lumen_dia_mm      = 2.0;
    in.length_mm         = 30.0;
    in.head_dia_mm       = 10.0;
    in.head_depth_mm     = 3.0;

    auto out = skill::cannulated_screw_lumen::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("lumen_through").get<bool>());
    EXPECT_NEAR(out.signature.pattern.at("lumen_dia_mm").get<double>(), 2.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("outer_dia_mm").get<double>(), 8.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("head_dia_mm").get<double>(), 10.0, 1e-6);
}
