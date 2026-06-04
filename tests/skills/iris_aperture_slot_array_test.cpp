// @lat: [[process/test-strategy#iris_aperture_slot_array]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/iris_aperture_slot_array.hpp"

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

skill::iris_aperture_slot_array::Input good(int faceId)
{
    skill::iris_aperture_slot_array::Input in;
    in.face_id              = faceId;
    in.center_x_mm          = 30.0;
    in.center_y_mm          = 30.0;
    in.bore_dia_mm          = 6.0;
    in.slot_count           = 12;
    in.slot_inner_radius_mm = 5.0;
    in.slot_outer_radius_mm = 9.0;
    in.slot_width_mm        = 0.6;
    in.slot_depth_mm        = 1.5;
    return in;
}

}  // namespace

// ─── 1. apply removes volume ≈ bore + N slots ─────────────────────────────
TEST(SkillIrisApertureSlotArray, ApplyRemovesVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);
    auto in = good(topId);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::iris_aperture_slot_array::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 5.0);   // at least 5 mm^3 removed
}

// ─── 2. validate rejects slot_count out of range ─────────────────────────
TEST(SkillIrisApertureSlotArray, ValidateRejectsSlotCount)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.slot_count = 3;   // < 4
    auto r = skill::iris_aperture_slot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT-COUNT") found = true;
    EXPECT_TRUE(found);

    in.slot_count = 40;  // > 36
    r = skill::iris_aperture_slot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 3. validate rejects bad slot radii ──────────────────────────────────
TEST(SkillIrisApertureSlotArray, ValidateRejectsBadRadii)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.slot_outer_radius_mm = in.slot_inner_radius_mm;   // equal — invalid
    auto r = skill::iris_aperture_slot_array::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT-RADIUS") found = true;
    EXPECT_TRUE(found);
}

// ─── 4. signature carries slot_count + is_periodic_pattern ───────────────
TEST(SkillIrisApertureSlotArray, SignatureCarriesPeriodicPattern)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::iris_aperture_slot_array::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_periodic_pattern"].get<bool>());
    EXPECT_EQ(out.signature.pattern["slot_count"].get<int>(), 12);
    EXPECT_EQ(out.signature.pattern["optical_feature_type"].get<std::string>(),
              std::string("iris_slot_array"));
}

// ─── 5. recognize replays history ────────────────────────────────────────
TEST(SkillIrisApertureSlotArray, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::iris_aperture_slot_array::apply(*stock, in);
    auto cands = skill::iris_aperture_slot_array::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("iris_aperture_slot_array"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
