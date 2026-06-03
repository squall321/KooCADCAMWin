// @lat: [[process/test-strategy#skill round-trip]]
//
// copy_body_with_transform — duplicate workpiece with combined rotation +
// translation; result is a TopoDS_Compound { original, copy }.
//
// Cases (5):
//   1. apply yields a compound with 2 solids whose volumes are equal.
//   2. pure-translation copy: bbox spans both originals.
//   3. DFM rejects identity transform.
//   4. DFM rejects zero-length rotation axis.
//   5. recognize identifies the duplicate (history-replay).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/copy_body_with_transform.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
int countSolids(const TopoDS_Shape& s)
{
    int n = 0;
    for (TopExp_Explorer e(s, TopAbs_SOLID); e.More(); e.Next()) ++n;
    return n;
}
}  // namespace

// ─── 1. compound with 2 equal-volume solids ────────────────────────────────
TEST(SkillCopyBodyWithTransform, ApplyProducesEqualVolumeDuplicate)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::copy_body_with_transform::Input in;
    in.translation_xyz      = { 50.0, 0.0, 0.0 };
    in.rotation_axis_origin = { 0.0, 0.0, 0.0 };
    in.rotation_axis_dir    = { 0.0, 0.0, 1.0 };
    in.rotation_angle_deg   = 0.0;

    auto out = skill::copy_body_with_transform::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_EQ(countSolids(out.workpiece->shape()), 2);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());

    std::vector<double> vols;
    for (TopExp_Explorer e(out.workpiece->shape(), TopAbs_SOLID); e.More(); e.Next()) {
        GProp_GProps p; BRepGProp::VolumeProperties(e.Current(), p);
        vols.push_back(p.Mass());
    }
    ASSERT_EQ(vols.size(), 2u);
    EXPECT_NEAR(vols[0], volBefore, volBefore * 1e-6);
    EXPECT_NEAR(vols[1], volBefore, volBefore * 1e-6);
}

// ─── 2. pure translation moves copy along +X ───────────────────────────────
TEST(SkillCopyBodyWithTransform, TranslationExpandsBbox)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::copy_body_with_transform::Input in;
    in.translation_xyz      = { 50.0, 0.0, 0.0 };
    in.rotation_axis_origin = { 0.0, 0.0, 0.0 };
    in.rotation_axis_dir    = { 0.0, 0.0, 1.0 };
    in.rotation_angle_deg   = 0.0;
    auto out = skill::copy_body_with_transform::apply(*stock, in);

    Bnd_Box bb;
    BRepBndLib::AddOptimal(out.workpiece->shape(), bb);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    bb.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_GT(xMax - xMin, 60.0);     // original 20 + 50 shift = 70 wide
}

// ─── 3. DFM rejects identity ───────────────────────────────────────────────
TEST(SkillCopyBodyWithTransform, ValidateRejectsIdentity)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::copy_body_with_transform::Input in;     // all defaults → identity
    auto r = skill::copy_body_with_transform::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::copy_body_with_transform::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects zero rotation axis ─────────────────────────────────────
TEST(SkillCopyBodyWithTransform, ValidateRejectsZeroAxis)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::copy_body_with_transform::Input in;
    in.translation_xyz   = { 1.0, 0.0, 0.0 };
    in.rotation_axis_dir = { 0.0, 0.0, 0.0 };
    in.rotation_angle_deg = 45.0;
    auto r = skill::copy_body_with_transform::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 5. recognize identifies duplicate ─────────────────────────────────────
TEST(SkillCopyBodyWithTransform, RecognizeIdentifiesDuplicate)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 10.0);
    skill::copy_body_with_transform::Input in;
    in.translation_xyz      = { 50.0, 0.0, 0.0 };
    in.rotation_axis_origin = { 0.0, 0.0, 0.0 };
    in.rotation_axis_dir    = { 0.0, 0.0, 1.0 };
    in.rotation_angle_deg   = 90.0;
    auto out   = skill::copy_body_with_transform::apply(*stock, in);
    auto cands = skill::copy_body_with_transform::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("copy_body_with_transform"));
    EXPECT_GT(cands[0].confidence, 0.85);
    EXPECT_NEAR(cands[0].recovered_params["rotation_angle_deg"].get<double>(),
                90.0, 1e-9);
}
