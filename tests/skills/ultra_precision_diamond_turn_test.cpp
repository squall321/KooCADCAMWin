// @lat: [[process/test-strategy#skill round-trip]]
//
// ultra_precision_diamond_turn — 5-case sweep covering:
//   1. apply() delegates geometry to turn_external (volume change)
//   2. ferrous material rejected (DFM-SPDT-FERROUS)
//   3. signature records kind + diamond tooling + target Ra
//   4. recognize() metadata replay
//   5. invalid input (end_z ≤ start_z) rejected

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ultra_precision_diamond_turn.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply delegates to turn_external — volume changes ────────────────
TEST(SkillUltraPrecisionDiamondTurn, ApplyChangesGeometry)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0, "aluminum_6061");
    const double volBefore = volumeOf(stock->shape());

    skill::ultra_precision_diamond_turn::Input in;
    in.start_z_mm      = 5.0;
    in.end_z_mm        = 15.0;
    in.final_dia_mm    = 34.0;
    in.roughing_passes = 1;
    in.tool_nose_r_mm  = 0.5;
    in.target_ra_nm    = 5.0;       // optical grade
    in.workpiece_material = "aluminum_6061";  // non-ferrous OK

    auto out = skill::ultra_precision_diamond_turn::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_LT(volumeOf(out.workpiece->shape()), volBefore);
    EXPECT_EQ(out.signature.skill_id,
              std::string("ultra_precision_diamond_turn"));
    EXPECT_EQ(out.workpiece->features().size(), 2u);   // turn_external + this
}

// ─── 2. Ferrous workpiece is rejected (DFM-SPDT-FERROUS) ─────────────────
TEST(SkillUltraPrecisionDiamondTurn, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0, "carbon_steel_1045");

    skill::ultra_precision_diamond_turn::Input in;
    in.start_z_mm      = 5.0;
    in.end_z_mm        = 15.0;
    in.final_dia_mm    = 34.0;
    in.roughing_passes = 1;
    in.target_ra_nm    = 5.0;
    in.workpiece_material = "carbon_steel_1045";   // ferrous → forbidden

    auto r = skill::ultra_precision_diamond_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SPDT-FERROUS"));
    EXPECT_THROW(skill::ultra_precision_diamond_turn::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records kind + diamond tooling + target Ra ─────────────
TEST(SkillUltraPrecisionDiamondTurn, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0, "aluminum_6061");

    skill::ultra_precision_diamond_turn::Input in;
    in.start_z_mm      = 4.0;
    in.end_z_mm        = 16.0;
    in.final_dia_mm    = 30.0;
    in.roughing_passes = 1;
    in.tool_nose_r_mm  = 0.25;
    in.target_ra_nm    = 3.0;
    in.workpiece_material = "aluminum_6061";

    auto out = skill::ultra_precision_diamond_turn::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ultra_precision_diamond_turn"));
    EXPECT_EQ(out.signature.pattern["strategy"].get<std::string>(),
              std::string("spdt"));
    EXPECT_NEAR(out.signature.pattern["target_ra_nm"].get<double>(),   3.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["tool_nose_r_mm"].get<double>(), 0.25, 1e-9);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("spdt_diamond_insert"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("single_crystal_diamond"));
    EXPECT_GE(out.signature.tooling.cutting_speed_sfm, 1000.0)
        << "SPDT runs at high surface speed";
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillUltraPrecisionDiamondTurn, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0, "aluminum_6061");

    skill::ultra_precision_diamond_turn::Input in;
    in.start_z_mm      = 5.0;
    in.end_z_mm        = 15.0;
    in.final_dia_mm    = 32.0;
    in.roughing_passes = 1;
    in.target_ra_nm    = 5.0;
    in.workpiece_material = "aluminum_6061";

    auto out   = skill::ultra_precision_diamond_turn::apply(*stock, in);
    auto cands = skill::ultra_precision_diamond_turn::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["target_ra_nm"].get<double>(),
                5.0, 1e-9);

    // Metadata-free → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::ultra_precision_diamond_turn::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Invalid Z range rejected ─────────────────────────────────────────
TEST(SkillUltraPrecisionDiamondTurn, ValidateRejectsInvertedZRange)
{
    auto stock = skill::createCylindricalStock(40.0, 20.0, "aluminum_6061");

    skill::ultra_precision_diamond_turn::Input in;
    in.start_z_mm      = 15.0;
    in.end_z_mm        = 5.0;        // inverted
    in.final_dia_mm    = 30.0;
    in.target_ra_nm    = 5.0;
    in.workpiece_material = "aluminum_6061";
    in.roughing_passes = 1;

    auto r = skill::ultra_precision_diamond_turn::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::ultra_precision_diamond_turn::apply(*stock, in),
                 skill::SkillError);
}
