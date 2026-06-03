// @lat: [[process/test-strategy#skill round-trip]]
//
// rib_along_path — add a polyline rib on top of a host face.
//
// Cases (5):
//   1. ApplyAddsVolume — result volume > original (additive feature).
//   2. ValidateRejectsShortPath — < 2 points → DFM error.
//   3. ValidateRejectsZeroThickness.
//   4. SignatureRecordsCompoundKind.
//   5. RecognizeReplaysHistory.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rib_along_path.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Find the +Z top face of a cuboid (largest area).
int findTopFace(const skill::Workpiece& wp)
{
    int    bestId   = -1;
    double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) { bestArea = a; bestId = i; }
    }
    return bestId;
}
}  // namespace

// ─── 1. Rib is additive — result volume strictly greater ──────────────────
TEST(SkillRibAlongPath, ApplyAddsVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::rib_along_path::Input in;
    in.host_face_id     = topId;
    in.path_polyline    = { { 5.0, 5.0 }, { 35.0, 25.0 } };  // diagonal
    in.rib_thickness_mm = 2.0;
    in.rib_height_mm    = 3.0;

    auto out = skill::rib_along_path::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());

    EXPECT_GT(volAfter, volBefore);
    EXPECT_GT(volAfter - volBefore, 1.0);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects path with < 2 points ─────────────────────────────
TEST(SkillRibAlongPath, ValidateRejectsShortPath)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::rib_along_path::Input in;
    in.host_face_id     = topId;
    in.path_polyline    = { { 5.0, 5.0 } };       // only 1 point
    in.rib_thickness_mm = 2.0;
    in.rib_height_mm    = 3.0;

    auto r = skill::rib_along_path::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::rib_along_path::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects zero thickness ───────────────────────────────────
TEST(SkillRibAlongPath, ValidateRejectsZeroThickness)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::rib_along_path::Input in;
    in.host_face_id     = topId;
    in.path_polyline    = { { 5.0, 5.0 }, { 35.0, 25.0 } };
    in.rib_thickness_mm = 0.0;
    in.rib_height_mm    = 3.0;

    auto r = skill::rib_along_path::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature records compound + kind ────────────────────────────────
TEST(SkillRibAlongPath, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::rib_along_path::Input in;
    in.host_face_id     = topId;
    in.path_polyline    = { { 5.0, 5.0 }, { 35.0, 5.0 } };
    in.rib_thickness_mm = 2.0;
    in.rib_height_mm    = 3.0;
    auto out = skill::rib_along_path::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("rib_along_path"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rib_along_path"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["segment_count"].get<int>(), 1);
}

// ─── 5. Recognize replays history ─────────────────────────────────────────
TEST(SkillRibAlongPath, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::rib_along_path::Input in;
    in.host_face_id     = topId;
    in.path_polyline    = { { 5.0, 5.0 }, { 35.0, 5.0 }, { 35.0, 25.0 } };
    in.rib_thickness_mm = 2.0;
    in.rib_height_mm    = 3.0;
    auto out = skill::rib_along_path::apply(*stock, in);

    auto cands = skill::rib_along_path::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("rib_along_path"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
