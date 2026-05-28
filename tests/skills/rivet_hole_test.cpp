// @lat: [[process/test-strategy#skill round-trip]]
//
// rivet_hole skill — 5-case sweep covering synthesis, DFM, standard-class
// lookup, recognition and through-hole property.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rivet_hole.hpp"

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

}  // namespace

// ─── 1. Apply: drills through the workpiece ───────────────────────────────
TEST(SkillRivetHole, ApplyCreatesThroughHole)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::rivet_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = 5.0;
    in.rivet_diameter_class = "M5";

    auto out = skill::rivet_hole::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = M_PI * 2.5 * 2.5 * 10.0;  // through hole
    const double diff = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("rivet_hole"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("rivet_drill"));
    EXPECT_EQ(out.signature.params["through_hole"].get<bool>(), true);
}

// ─── 2. DFM: sub-2 mm diameter rejected ───────────────────────────────────
TEST(SkillRivetHole, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::rivet_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.diameter_mm   = 1.5;          // < 2.0

    auto r = skill::rivet_hole::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::rivet_hole::apply(*stock, in), skill::SkillError);
}

// ─── 3. Standard-class lookup matches well-known sizes ────────────────────
TEST(SkillRivetHole, StandardClassLookup)
{
    // Metric metric matches.
    EXPECT_EQ(skill::rivet_hole::classFromDiameter(2.0),  std::string("M2"));
    EXPECT_EQ(skill::rivet_hole::classFromDiameter(5.0),  std::string("M5"));
    EXPECT_EQ(skill::rivet_hole::classFromDiameter(6.0),  std::string("M6"));
    EXPECT_EQ(skill::rivet_hole::classFromDiameter(10.0), std::string("M10"));

    // 3/16" imperial.
    EXPECT_EQ(skill::rivet_hole::classFromDiameter(4.7625),
              std::string("ANSI_3_16in"));

    // Non-standard returns empty.
    EXPECT_TRUE(skill::rivet_hole::classFromDiameter(7.3).empty());

    // Inverse lookup.
    EXPECT_NEAR(skill::rivet_hole::diameterFromClass("M5"), 5.0, 1e-6);
    EXPECT_NEAR(skill::rivet_hole::diameterFromClass("ANSI_3_16in"), 4.7625, 1e-6);
    EXPECT_NEAR(skill::rivet_hole::diameterFromClass("nonsense"), 0.0, 1e-6);
}

// ─── 4. Recognize matches a standard rivet hole ───────────────────────────
TEST(SkillRivetHole, RecognizeMatchesStandard)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::rivet_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.diameter_mm   = 4.0;

    auto out = skill::rivet_hole::apply(*stock, in);
    auto cands = skill::rivet_hole::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), 4.0, 0.05);
    EXPECT_EQ(c.recovered_params["rivet_diameter_class"].get<std::string>(),
              std::string("M4"));
    EXPECT_EQ(c.recovered_params["through_hole"].get<bool>(), true);
}

// ─── 5. Non-standard diameter NOT recognized as rivet_hole ────────────────
TEST(SkillRivetHole, RecognizeRejectsNonStandard)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::rivet_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.diameter_mm   = 7.3;   // not a standard rivet size
    in.rivet_diameter_class = "";

    auto out = skill::rivet_hole::apply(*stock, in);
    auto cands = skill::rivet_hole::recognize(*out.workpiece);
    EXPECT_TRUE(cands.empty())
        << "rivet_hole::recognize should reject non-standard diameter 7.3 mm";
}

// ─── 6. Apply always produces a through hole regardless of inputs ─────────
TEST(SkillRivetHole, AlwaysThrough)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    skill::rivet_hole::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.diameter_mm   = 3.0;

    auto out = skill::rivet_hole::apply(*stock, in);
    EXPECT_EQ(out.signature.params["through_hole"].get<bool>(), true);

    auto cands = skill::rivet_hole::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].recovered_params["through_hole"].get<bool>(), true);
}
