// @lat: [[process/test-strategy#optical_axis_alignment_pin_pair]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/optical_axis_alignment_pin_pair.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

skill::optical_axis_alignment_pin_pair::Input good(int faceId)
{
    skill::optical_axis_alignment_pin_pair::Input in;
    in.face_id    = faceId;
    in.pin_a_x_mm = 15.0;
    in.pin_a_y_mm = 25.0;
    in.pin_b_x_mm = 45.0;
    in.pin_b_y_mm = 25.0;
    in.pin_dia_mm = 4.0;
    in.pin_depth_mm = 8.0;
    in.tolerance_class = "H7";
    return in;
}

}  // namespace

// ─── 1. apply removes ≈ 2× pin cylinder volume ───────────────────────────
TEST(SkillOpticalAxisAlignmentPinPair, ApplyRemovesTwoCylinders)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 15.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);
    auto in = good(topId);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::optical_axis_alignment_pin_pair::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());
    const double diff = v0 - v1;

    const double r = in.pin_dia_mm / 2.0;
    const double approx = 2.0 * M_PI * r * r * in.pin_depth_mm;
    EXPECT_NEAR(diff, approx, approx * 0.25);
}

// ─── 2. validate rejects bad tolerance class ─────────────────────────────
TEST(SkillOpticalAxisAlignmentPinPair, ValidateRejectsBadTolClass)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 15.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.tolerance_class = "H8";

    auto r = skill::optical_axis_alignment_pin_pair::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PIN-TOL") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::optical_axis_alignment_pin_pair::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects pin centres too close ───────────────────────────
TEST(SkillOpticalAxisAlignmentPinPair, ValidateRejectsClosePins)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 15.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.pin_b_x_mm = in.pin_a_x_mm + 1.0;   // < 2×4 mm = 8 mm spacing
    auto r = skill::optical_axis_alignment_pin_pair::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PIN-SPACING") found = true;
    EXPECT_TRUE(found);
}

// ─── 4. signature carries tolerance + is_compound ────────────────────────
TEST(SkillOpticalAxisAlignmentPinPair, SignatureCarriesTolClass)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 15.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.tolerance_class = "H6";

    auto out = skill::optical_axis_alignment_pin_pair::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["tolerance_class"].get<std::string>(),
              std::string("H6"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 2);
    EXPECT_EQ(out.signature.pattern["optical_feature_type"].get<std::string>(),
              std::string("alignment_pin_pair"));
}

// ─── 5. recognize replays + H6 final dia tighter than H7 ─────────────────
TEST(SkillOpticalAxisAlignmentPinPair, RecognizeAndH6TighterThanH7)
{
    auto stock = skill::createCuboidStock(60.0, 50.0, 15.0);
    const int topId = findTopFace(*stock);

    auto inH6 = good(topId);  inH6.tolerance_class = "H6";
    auto inH7 = good(topId);  inH7.tolerance_class = "H7";
    auto outH6 = skill::optical_axis_alignment_pin_pair::apply(*stock, inH6);
    auto outH7 = skill::optical_axis_alignment_pin_pair::apply(*stock, inH7);

    EXPECT_LT(outH6.signature.pattern["final_dia_mm"].get<double>(),
              outH7.signature.pattern["final_dia_mm"].get<double>());

    auto cands = skill::optical_axis_alignment_pin_pair::recognize(*outH6.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_GE(cands.front().confidence, 0.9);
}
