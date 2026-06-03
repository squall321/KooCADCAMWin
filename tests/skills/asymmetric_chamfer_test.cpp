// @lat: [[process/test-strategy#skill round-trip]]
//
// asymmetric_chamfer — apply + DFM + recognize sanity checks.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/asymmetric_chamfer.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: asymmetric chamfer on the top rim of a cuboid ──────────────
TEST(SkillAsymmetricChamfer, ApplyTopRimSucceeds)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::asymmetric_chamfer::Input in;
    in.edge_z_band_min = 9.9;
    in.edge_z_band_max = 10.1;
    in.distance_a_mm   = 1.0;
    in.distance_b_mm   = 2.0;

    auto out = skill::asymmetric_chamfer::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const int chamfered =
        out.signature.pattern["edge_count_chamfered"].get<int>();
    EXPECT_GT(chamfered, 0);

    // Distinct chamfer face count > 0 — new planar faces were added.
    const int distinctFaces =
        out.signature.pattern["distinct_chamfer_face_count"].get<int>();
    EXPECT_GT(distinctFaces, 0);

    // Material removed.
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("asymmetric_chamfer"));
}

// ─── 2. Validate rejects bad distances ───────────────────────────────────
TEST(SkillAsymmetricChamfer, ValidateRejectsBadDistances)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::asymmetric_chamfer::Input in;
    in.edge_z_band_min = 4.9;
    in.edge_z_band_max = 5.1;
    in.distance_a_mm   = -1.0;   // bad
    in.distance_b_mm   =  1.0;

    auto r = skill::asymmetric_chamfer::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::asymmetric_chamfer::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. No matching edges throws ─────────────────────────────────────────
TEST(SkillAsymmetricChamfer, NoMatchingEdgesThrows)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::asymmetric_chamfer::Input in;
    in.edge_z_band_min = 100.0;
    in.edge_z_band_max = 110.0;
    in.distance_a_mm   = 1.0;
    in.distance_b_mm   = 1.0;

    EXPECT_THROW(skill::asymmetric_chamfer::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. Recognize: history replay ─────────────────────────────────────────
TEST(SkillAsymmetricChamfer, RecognizeHistoryReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::asymmetric_chamfer::Input in;
    in.edge_z_band_min = 9.9;
    in.edge_z_band_max = 10.1;
    in.distance_a_mm   = 0.8;
    in.distance_b_mm   = 1.5;

    auto out = skill::asymmetric_chamfer::apply(*stock, in);
    auto cands = skill::asymmetric_chamfer::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["distance_a_mm"].get<double>(),
                in.distance_a_mm, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["distance_b_mm"].get<double>(),
                in.distance_b_mm, 1e-6);
}
