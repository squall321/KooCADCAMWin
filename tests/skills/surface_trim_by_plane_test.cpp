// @lat: [[process/test-strategy#skill round-trip]]
//
// surface_trim_by_plane — cut workpiece with a half-space.
//
// Cases (5):
//   1. ApplyShrinksVolume — trimming a Z-mid plane (keep above) halves volume.
//   2. ValidateRejectsZeroNormal.
//   3. ValidateRejectsMissPlane.
//   4. ValidateRejectsBadKeepSide.
//   5. SignatureAndRecognizeRoundtrip.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/surface_trim_by_plane.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply: trimming at Z = 5 keeps roughly half a 40×30×10 box ────────
TEST(SkillSurfaceTrimByPlane, ApplyShrinksVolumeByHalf)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double volBefore = volumeOf(stock->shape());     // 12000

    skill::surface_trim_by_plane::Input in;
    in.plane_origin = { 20.0, 15.0, 5.0 };
    in.plane_normal = {  0.0,  0.0, 1.0 };
    in.keep_side    = "above";

    auto out = skill::surface_trim_by_plane::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_NEAR(volAfter, volBefore * 0.5, volBefore * 0.05);
}

// ─── 2. DFM rejects zero normal ───────────────────────────────────────────
TEST(SkillSurfaceTrimByPlane, ValidateRejectsZeroNormal)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    skill::surface_trim_by_plane::Input in;
    in.plane_origin = { 15.0, 15.0, 5.0 };
    in.plane_normal = { 0.0, 0.0, 0.0 };
    auto r = skill::surface_trim_by_plane::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::surface_trim_by_plane::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. DFM rejects plane that misses the bbox ────────────────────────────
TEST(SkillSurfaceTrimByPlane, ValidateRejectsMissPlane)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    skill::surface_trim_by_plane::Input in;
    in.plane_origin = { 15.0, 15.0, 500.0 };
    in.plane_normal = { 0.0, 0.0, 1.0 };
    auto r = skill::surface_trim_by_plane::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MISS") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 4. DFM rejects bad keep_side string ──────────────────────────────────
TEST(SkillSurfaceTrimByPlane, ValidateRejectsBadKeepSide)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 10.0);
    skill::surface_trim_by_plane::Input in;
    in.plane_origin = { 15.0, 15.0, 5.0 };
    in.plane_normal = { 0.0, 0.0, 1.0 };
    in.keep_side    = "north";
    auto r = skill::surface_trim_by_plane::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-KEEP-SIDE") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 5. signature + recognize roundtrip ───────────────────────────────────
TEST(SkillSurfaceTrimByPlane, SignatureAndRecognizeRoundtrip)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    skill::surface_trim_by_plane::Input in;
    in.plane_origin = { 20.0, 15.0, 5.0 };
    in.plane_normal = { 0.0,  0.0, 1.0 };
    in.keep_side    = "above";
    auto out = skill::surface_trim_by_plane::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("surface_trim_by_plane"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["source_face_count"].get<int>(), 1);

    auto cands = skill::surface_trim_by_plane::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("surface_trim_by_plane"));
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
}
