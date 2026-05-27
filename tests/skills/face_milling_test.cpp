// @lat: [[process/test-strategy#skill round-trip]]
//
// face_milling skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/face_milling.hpp"

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

// ─── 1. Apply: skim 1 mm off top face ─────────────────────────────────────
TEST(SkillFaceMilling, ApplySkimRemovesUniformLayer)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const double topBefore = topZ(stock->shape());

    skill::face_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.depth_mm   = 1.0;

    auto out = skill::face_milling::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double topAfter = topZ(out.workpiece->shape());

    // Volume removed ≈ 50 × 50 × 1 = 2500 mm³.
    EXPECT_NEAR(volBefore - volAfter, 2500.0, 2500.0 * 0.02);
    // Top face shifted down by 1 mm.
    EXPECT_NEAR(topBefore - topAfter, 1.0, 1e-3);

    EXPECT_EQ(out.signature.skill_id, std::string("face_milling"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate: zero / negative depth rejected ──────────────────────────
TEST(SkillFaceMilling, ValidateRejectsZeroDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::face_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.depth_mm   = 0.0;

    auto r = skill::face_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);

    EXPECT_THROW(skill::face_milling::apply(*stock, in), skill::SkillError);
}

// ─── 3. Validate: deep skim emits warning, not error ──────────────────────
TEST(SkillFaceMilling, ValidateWarnsOnDeepSkim)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 20.0);

    skill::face_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.depth_mm   = 6.0;        // > 5 mm → warning

    auto r = skill::face_milling::validate(*stock, in);
    EXPECT_TRUE(r.passed);       // warning, not error
    bool foundWarn = false;
    for (const auto& f : r.findings)
        if (f.severity == "warning") { foundWarn = true; break; }
    EXPECT_TRUE(foundWarn);

    // apply() should still succeed.
    EXPECT_NO_THROW(skill::face_milling::apply(*stock, in));
}

// ─── 4. Recognize: finds top face as a candidate ──────────────────────────
TEST(SkillFaceMilling, RecognizeFindsTopFace)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::face_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.depth_mm   = 1.0;
    auto out = skill::face_milling::apply(*stock, in);

    auto candidates = skill::face_milling::recognize(*out.workpiece);
    ASSERT_FALSE(candidates.empty());
    // At least one candidate should have a +Z normal (the new top face).
    bool foundTop = false;
    for (const auto& c : candidates) {
        const auto& n = c.recovered_params["entry_face_normal"];
        if (n.is_array() && n.size() == 3 && n[2].get<double>() > 0.9) {
            foundTop = true; break;
        }
    }
    EXPECT_TRUE(foundTop);
}

// ─── 5. Round-trip: face count and volume stable on re-apply ──────────────
TEST(SkillFaceMilling, RoundTripStable)
{
    auto stock1 = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::face_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.depth_mm   = 1.5;
    auto out1 = skill::face_milling::apply(*stock1, in);

    auto stock2 = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out2 = skill::face_milling::apply(*stock2, in);

    EXPECT_EQ(out1.workpiece->faceCount(), out2.workpiece->faceCount());
    EXPECT_EQ(out1.workpiece->edgeCount(), out2.workpiece->edgeCount());
    EXPECT_NEAR(volumeOf(out1.workpiece->shape()),
                volumeOf(out2.workpiece->shape()), 1e-3);
}
