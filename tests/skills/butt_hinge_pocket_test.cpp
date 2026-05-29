// @lat: [[process/test-strategy#compound macro]]
//
// butt_hinge_pocket — compound: pocket + 2 screw counterbores + chamfer.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/butt_hinge_pocket.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. Apply removes ~ pocket + 2 screws + chamfer volumes ────────────────
TEST(SkillButtHingePocket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 10.0);

    skill::butt_hinge_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm       = 50.0;
    in.center_y_mm       = 30.0;
    in.axis_dir          = gp_Dir(0, 0, -1);
    in.hinge_length_mm   = 60.0;
    in.hinge_width_mm    = 30.0;
    in.leaf_thickness_mm = 1.5;

    const int origFaces = stock->faceCount();
    auto out = skill::butt_hinge_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected volume removed:
    //  pocket = 60 × 30 × 1.5 = 2700
    //  2 screw counterbores ≈ 2 × (π × 1.75² × 6.0 + π × (3.75² − 1.75²) × 1.8) ≈ 178
    //  chamfer ≈ 60 × 0.6 × 0.6 × 0.5 ≈ 10.8
    const double pocketV = 60.0 * 30.0 * 1.5;
    const double oneScrewV = M_PI * 1.75 * 1.75 * 6.0
                           + M_PI * (3.75 * 3.75 - 1.75 * 1.75) * 1.8;
    const double chamferV = 60.0 * 0.6 * 0.6 * 0.5;
    const double expected = pocketV + 2.0 * oneScrewV + chamferV;
    const double removed  = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.15);

    // Face count must grow — compound removes a complex topology.
    EXPECT_GT(out.workpiece->faceCount(), origFaces);
}

// ─── 2. Validate rejects out-of-catalog hinge length ───────────────────────
TEST(SkillButtHingePocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 60.0, 10.0);

    skill::butt_hinge_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm       = 100.0;
    in.center_y_mm       = 30.0;
    in.hinge_length_mm   = 180.0;   // > 150 mm → out of catalog
    in.hinge_width_mm    = 30.0;

    auto r = skill::butt_hinge_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HINGE-LENGTH") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::butt_hinge_pocket::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound metadata ────────────────────────────────
TEST(SkillButtHingePocket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 10.0);

    skill::butt_hinge_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm       = 50.0;
    in.center_y_mm       = 30.0;
    in.hinge_length_mm   = 60.0;
    in.hinge_width_mm    = 30.0;
    in.leaf_thickness_mm = 1.5;
    auto out = skill::butt_hinge_pocket::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("butt_hinge_pocket"));
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("butt_hinge_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(out.signature.pattern.at("screw_positions").size(), 2u);
}

// ─── 4. Recognize replays from history with confidence 1.0 ─────────────────
TEST(SkillButtHingePocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 10.0);

    skill::butt_hinge_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm       = 50.0;
    in.center_y_mm       = 30.0;
    in.hinge_length_mm   = 60.0;
    in.hinge_width_mm    = 30.0;
    auto out = skill::butt_hinge_pocket::apply(*stock, in);

    auto cands = skill::butt_hinge_pocket::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("hinge_length_mm").get<double>(), 60.0);
}

// ─── 5. Specific: screw pilot dia matches M3.5 catalog ─────────────────────
TEST(SkillButtHingePocket, ScrewPilotMatchesM35Catalog)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 10.0);

    skill::butt_hinge_pocket::Input in;
    in.entry_face        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm       = 50.0;
    in.center_y_mm       = 30.0;
    in.hinge_length_mm   = 60.0;
    in.hinge_width_mm    = 30.0;
    auto out = skill::butt_hinge_pocket::apply(*stock, in);

    // M3.5 wood screw: pilot 3.5 mm, seat 7.5 mm (seat > pilot mandatory).
    const double pilot = out.signature.pattern.at("screw_pilot_dia_mm").get<double>();
    const double seat  = out.signature.pattern.at("screw_seat_dia_mm").get<double>();
    EXPECT_NEAR(pilot, 3.5, 1e-6);
    EXPECT_NEAR(seat,  7.5, 1e-6);
    EXPECT_GT(seat, pilot);
}
