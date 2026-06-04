// @lat: [[process/test-strategy#c_mount_lens_thread]]
//
// c_mount_lens_thread — 5 tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/c_mount_lens_thread.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

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

// ─── 1. apply removes a measurable volume ────────────────────────────────
TEST(SkillCMountLensThread, ApplyRemovesVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    skill::c_mount_lens_thread::Input in;
    in.face_id     = topId;
    in.center_x_mm = 30.0;
    in.center_y_mm = 30.0;
    in.mount_style = "C";

    const double v0 = volumeOf(stock->shape());
    auto out = skill::c_mount_lens_thread::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 1000.0) << "expected > 1 cm^3 removed for C-mount";
    EXPECT_EQ(out.signature.skill_id, std::string("c_mount_lens_thread"));
}

// ─── 2. validate rejects unknown mount_style ─────────────────────────────
TEST(SkillCMountLensThread, ValidateRejectsUnknownStyle)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);
    const int topId = findTopFace(*stock);

    skill::c_mount_lens_thread::Input in;
    in.face_id     = topId;
    in.center_x_mm = 30.0;
    in.center_y_mm = 30.0;
    in.mount_style = "M12";

    auto r = skill::c_mount_lens_thread::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-MOUNT-STYLE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::c_mount_lens_thread::apply(*stock, in), skill::SkillError);
}

// ─── 3. signature carries mount_style + is_compound=true ─────────────────
TEST(SkillCMountLensThread, SignatureCarriesMountAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);
    const int topId = findTopFace(*stock);

    skill::c_mount_lens_thread::Input in;
    in.face_id     = topId;
    in.center_x_mm = 30.0;
    in.center_y_mm = 30.0;
    in.mount_style = "CS";

    auto out = skill::c_mount_lens_thread::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("c_mount_lens_thread"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["mount_style"].get<std::string>(), std::string("CS"));
    EXPECT_EQ(out.signature.pattern["optical_feature_type"].get<std::string>(),
              std::string("lens_thread_mount"));
    EXPECT_NEAR(out.signature.pattern["flange_distance_mm"].get<double>(), 12.5, 0.01);
}

// ─── 4. recognize replays C-mount candidate ──────────────────────────────
TEST(SkillCMountLensThread, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 25.0);
    const int topId = findTopFace(*stock);

    skill::c_mount_lens_thread::Input in;
    in.face_id     = topId;
    in.center_x_mm = 30.0;
    in.center_y_mm = 30.0;
    in.mount_style = "C";

    auto out = skill::c_mount_lens_thread::apply(*stock, in);
    auto cands = skill::c_mount_lens_thread::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("c_mount_lens_thread"));
    EXPECT_GE(cands.front().confidence, 0.9);
}

// ─── 5. lookupMount returns canonical spec for each style ────────────────
TEST(SkillCMountLensThread, LookupMountCanonicalSpecs)
{
    skill::c_mount_lens_thread::MountSpec sp{};
    ASSERT_TRUE(skill::c_mount_lens_thread::lookupMount("C", sp));
    EXPECT_NEAR(sp.od_mm, 25.4, 1e-3);
    EXPECT_NEAR(sp.flange_distance_mm, 17.526, 1e-3);

    ASSERT_TRUE(skill::c_mount_lens_thread::lookupMount("CS", sp));
    EXPECT_NEAR(sp.flange_distance_mm, 12.5, 1e-3);

    ASSERT_TRUE(skill::c_mount_lens_thread::lookupMount("M42", sp));
    EXPECT_NEAR(sp.od_mm, 42.0, 1e-3);
    EXPECT_NEAR(sp.pitch_mm, 1.0, 1e-3);

    ASSERT_TRUE(skill::c_mount_lens_thread::lookupMount("M52", sp));
    EXPECT_NEAR(sp.od_mm, 52.0, 1e-3);
    EXPECT_NEAR(sp.pitch_mm, 0.75, 1e-3);

    EXPECT_FALSE(skill::c_mount_lens_thread::lookupMount("BNC", sp));
}
