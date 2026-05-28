// @lat: [[process/test-strategy#skill round-trip]]
//
// surface_grind — precision flatness/finish on a planar face.  Wraps
// face_milling internally; differs only in signature + tooling metadata.
//
// Tests (5):
//   1. apply() removes the requested thin layer (delegated face_milling).
//   2. DFM rejects out-of-range removal_mm (below 0.005, above 0.2).
//   3. apply() carries the surface_grind signature/tooling, not face_milling.
//   4. recognize() returns empty (metadata-only distinction).
//   5. Composed: face_milling first, then surface_grind for final finish.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/face_milling.hpp"
#include "skills/surface_grind.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
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

double topZ(const TopoDS_Shape& s)
{
    Bnd_Box b;
    BRepBndLib::AddOptimal(s, b);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    b.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    return zMax;
}

}  // namespace

// ─── 1. Apply: removes the requested thin layer ──────────────────────────
TEST(SkillSurfaceGrind, ApplyRemovesThinLayer)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const double topBefore = topZ(stock->shape());

    skill::surface_grind::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.removal_mm = 0.05;
    in.surface_finish = "ra_0.4";

    auto out = skill::surface_grind::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double topAfter = topZ(out.workpiece->shape());

    // Volume removed ≈ 50 × 50 × 0.05 = 125 mm³.
    EXPECT_NEAR(volBefore - volAfter, 125.0, 125.0 * 0.05);
    // Top face shifted down by removal_mm.
    EXPECT_NEAR(topBefore - topAfter, 0.05, 1e-3);

    EXPECT_EQ(out.signature.skill_id, std::string("surface_grind"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.workpiece->features().front().skill_id,
              std::string("surface_grind"));
}

// ─── 2. DFM rejects out-of-range removal_mm ──────────────────────────────
TEST(SkillSurfaceGrind, ValidateRejectsOutOfRange)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Too small (< 0.005 mm)
    {
        skill::surface_grind::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.removal_mm = 0.001;
        auto r = skill::surface_grind::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::surface_grind::apply(*stock, in), skill::SkillError);
    }

    // Too large (> 0.2 mm)
    {
        skill::surface_grind::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.removal_mm = 0.5;
        auto r = skill::surface_grind::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::surface_grind::apply(*stock, in), skill::SkillError);
    }

    // Zero / negative
    {
        skill::surface_grind::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.removal_mm = 0.0;
        auto r = skill::surface_grind::validate(*stock, in);
        EXPECT_FALSE(r.passed);
    }
}

// ─── 3. Apply carries grinding-specific metadata ─────────────────────────
TEST(SkillSurfaceGrind, ApplyCarriesGrindingMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::surface_grind::Input in;
    in.entry_face     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm     = 0.02;
    in.surface_finish = "ra_0.2";

    auto out = skill::surface_grind::apply(*stock, in);

    // Tooling identifies as grinding wheel, not end mill.
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("grinding_wheel"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("aluminum_oxide"));
    EXPECT_EQ(out.signature.tooling.flute_count, 0);
    // Grinder runs at high SFM.
    EXPECT_GT(out.signature.tooling.cutting_speed_sfm, 3000.0);

    // Pattern carries surface_grind kind and parameters.
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("surface_grind"));
    EXPECT_NEAR(out.signature.pattern["removal_mm"].get<double>(), 0.02, 1e-9);
    EXPECT_EQ(out.signature.pattern["surface_finish"].get<std::string>(),
              std::string("ra_0.2"));
}

// ─── 4. recognize() returns empty (metadata-only differentiation) ────────
TEST(SkillSurfaceGrind, RecognizeReturnsEmpty)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::surface_grind::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.removal_mm = 0.05;

    auto out = skill::surface_grind::apply(*stock, in);
    auto cands = skill::surface_grind::recognize(*out.workpiece);

    EXPECT_TRUE(cands.empty())
        << "surface_grind cannot be uniquely recognized from geometry alone";
}

// ─── 5. Composed: face_milling then surface_grind ────────────────────────
TEST(SkillSurfaceGrind, ComposedWithFaceMilling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double topStart = topZ(stock->shape());

    // Step 1: coarse face_milling 0.5 mm
    skill::face_milling::Input fmIn;
    fmIn.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    fmIn.depth_mm   = 0.5;
    auto milled = skill::face_milling::apply(*stock, fmIn);

    // Step 2: fine surface_grind 0.05 mm
    skill::surface_grind::Input sgIn;
    sgIn.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    sgIn.removal_mm = 0.05;
    auto ground = skill::surface_grind::apply(*milled.workpiece, sgIn);

    EXPECT_NEAR(topStart - topZ(ground.workpiece->shape()), 0.55, 1e-2);
    EXPECT_EQ(ground.signature.skill_id, std::string("surface_grind"));
}
