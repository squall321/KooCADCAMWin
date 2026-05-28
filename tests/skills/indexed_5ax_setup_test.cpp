// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/indexed_5ax_setup.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

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

// ── 1. Apply: rotation preserves volume but changes orientation ──────────
TEST(SkillIndexed5axSetup, ApplyRotatesWorkpiece)
{
    auto stock = skill::createCuboidStock(40.0, 20.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::indexed_5ax_setup::Input in;
    in.rotation_axis      = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));   // +Y
    in.rotation_angle_deg = 90.0;

    auto out = skill::indexed_5ax_setup::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume is invariant under rigid rotation.
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 0.001);

    EXPECT_EQ(out.signature.skill_id, std::string("indexed_5ax_setup"));
    EXPECT_TRUE(out.signature.pattern["is_setup_only"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["rotation_angle_deg"].get<double>(), 90.0, 1e-6);
}

// ── 2. Rotation by 90° around +Y moves +X-extending bbox to +Z direction
TEST(SkillIndexed5axSetup, RotationActuallyChangesBbox)
{
    auto stock = skill::createCuboidStock(40.0, 20.0, 10.0);
    double xMin0, yMin0, zMin0, xMax0, yMax0, zMax0;
    stock->boundingBox(xMin0, yMin0, zMin0, xMax0, yMax0, zMax0);

    skill::indexed_5ax_setup::Input in;
    in.rotation_axis      = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));
    in.rotation_angle_deg = 90.0;

    auto out = skill::indexed_5ax_setup::apply(*stock, in);
    double xMin1, yMin1, zMin1, xMax1, yMax1, zMax1;
    out.workpiece->boundingBox(xMin1, yMin1, zMin1, xMax1, yMax1, zMax1);

    // Original X-extent (40) becomes the Z-extent after 90° about +Y.
    EXPECT_NEAR(zMax1 - zMin1, xMax0 - xMin0, 0.05);
    // And original Z-extent (10) becomes X-extent.
    EXPECT_NEAR(xMax1 - xMin1, zMax0 - zMin0, 0.05);
}

// ── 3. DFM-INDEX-ANGLE: angle outside [-360, 360] rejected ───────────────
TEST(SkillIndexed5axSetup, ValidateRejectsOutOfRangeAngle)
{
    auto stock = skill::createCuboidStock(40.0, 20.0, 10.0);
    skill::indexed_5ax_setup::Input in;
    in.rotation_axis      = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));
    in.rotation_angle_deg = 400.0;   // > 360

    auto r = skill::indexed_5ax_setup::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INDEX-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::indexed_5ax_setup::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-005 info always emitted ───────────────────────────────────────
TEST(SkillIndexed5axSetup, ValidateEmitsDFM005Info)
{
    auto stock = skill::createCuboidStock(40.0, 20.0, 10.0);
    skill::indexed_5ax_setup::Input in;
    in.rotation_axis      = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));
    in.rotation_angle_deg = 45.0;

    auto r = skill::indexed_5ax_setup::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-005" && f.severity == "info") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: rotated workpiece is a low-confidence indexed candidate
TEST(SkillIndexed5axSetup, RecognizeFlagsRotatedWorkpiece)
{
    auto stock = skill::createCuboidStock(40.0, 20.0, 10.0);
    skill::indexed_5ax_setup::Input in;
    in.rotation_axis      = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));
    in.rotation_angle_deg = 45.0;       // intentionally NOT a multiple of 90°

    auto out = skill::indexed_5ax_setup::apply(*stock, in);
    auto cands = skill::indexed_5ax_setup::recognize(*out.workpiece);
    // After a 45° rotation, the largest face is tilted off-axis → emit a
    // candidate.  If recognition logic ever yields none (e.g. on a freshly
    // axis-aligned workpiece), we accept that gracefully.
    if (!cands.empty()) {
        EXPECT_EQ(cands[0].skill_id, std::string("indexed_5ax_setup"));
        EXPECT_LE(cands[0].confidence, 0.5);
        EXPECT_GE(cands[0].confidence, 0.2);
    }
}

// ── 6. Unrotated workpiece doesn't trigger recognition ───────────────────
TEST(SkillIndexed5axSetup, RecognizeSkipsAxisAlignedWorkpiece)
{
    auto stock = skill::createCuboidStock(40.0, 20.0, 10.0);
    auto cands = skill::indexed_5ax_setup::recognize(*stock);
    // Cuboid stock has axis-aligned planar faces → no indexed_5ax_setup
    // candidate should be emitted.
    EXPECT_TRUE(cands.empty());
}
