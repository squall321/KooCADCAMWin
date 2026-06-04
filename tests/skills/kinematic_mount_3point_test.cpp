// @lat: [[process/test-strategy#kinematic_mount_3point]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/kinematic_mount_3point.hpp"

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

skill::kinematic_mount_3point::Input good(int faceId)
{
    skill::kinematic_mount_3point::Input in;
    in.face_id       = faceId;
    in.seat0_x_mm    = 15.0;
    in.seat0_y_mm    = 15.0;
    in.seat1_x_mm    = 45.0;
    in.seat1_y_mm    = 15.0;
    in.seat2_x_mm    = 30.0;
    in.seat2_y_mm    = 45.0;
    in.seat_kind     = "sphere_pocket";
    in.sphere_dia_mm = 6.35;
    return in;
}

}  // namespace

// ─── 1. apply removes positive volume ────────────────────────────────────
TEST(SkillKinematicMount3Point, ApplyRemovesVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);
    auto in = good(topId);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::kinematic_mount_3point::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 10.0) << "expected at least 10 mm^3 removed";
    EXPECT_EQ(out.signature.skill_id, std::string("kinematic_mount_3point"));
}

// ─── 2. validate rejects unknown seat_kind ───────────────────────────────
TEST(SkillKinematicMount3Point, ValidateRejectsSeatKind)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.seat_kind = "magnet_pocket";

    auto r = skill::kinematic_mount_3point::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEAT-KIND") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::kinematic_mount_3point::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects duplicate seat positions ────────────────────────
TEST(SkillKinematicMount3Point, ValidateRejectsDuplicateSeats)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.seat1_x_mm = in.seat0_x_mm;
    in.seat1_y_mm = in.seat0_y_mm;
    auto r = skill::kinematic_mount_3point::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SEATS") found = true;
    EXPECT_TRUE(found);
}

// ─── 4. signature carries seat_count=3 + is_compound ─────────────────────
TEST(SkillKinematicMount3Point, SignatureCarriesSeats)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::kinematic_mount_3point::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["seat_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["optical_feature_type"].get<std::string>(),
              std::string("kinematic_mount"));
    EXPECT_EQ(out.signature.pattern["seat_kind_primary"].get<std::string>(),
              std::string("sphere_pocket"));
}

// ─── 5. recognize replays history ────────────────────────────────────────
TEST(SkillKinematicMount3Point, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::kinematic_mount_3point::apply(*stock, in);
    auto cands = skill::kinematic_mount_3point::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("kinematic_mount_3point"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
