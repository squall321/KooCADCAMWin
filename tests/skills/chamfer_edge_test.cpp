// @lat: [[process/test-strategy#skill round-trip]]
//
// chamfer_edge skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/chamfer_edge.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
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

// ─── 1. Apply: chamfer top rim of cuboid ─────────────────────────────────
TEST(SkillChamferEdge, ApplyTopRimSucceeds)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const int facesBefore  = stock->faceCount();
    const double volBefore = volumeOf(stock->shape());

    skill::chamfer_edge::Input in;
    in.edge_selector  = skill::chamfer_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in.chamfer_size_mm = 1.0;
    in.angle_deg       = 45.0;

    auto out = skill::chamfer_edge::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);

    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("chamfer_edge"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate: sub-min chamfer rejected ───────────────────────────────
TEST(SkillChamferEdge, ValidateRejectsSubMinChamfer)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::chamfer_edge::Input in;
    in.edge_selector  = skill::chamfer_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in.chamfer_size_mm = 0.05;     // < 0.1 mm
    in.angle_deg       = 45.0;

    auto r = skill::chamfer_edge::validate(*stock, in);
    EXPECT_FALSE(r.passed);

    EXPECT_THROW(skill::chamfer_edge::apply(*stock, in), skill::SkillError);
}

// ─── 3. Validate: out-of-range angle rejected ─────────────────────────────
TEST(SkillChamferEdge, ValidateRejectsBadAngle)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::chamfer_edge::Input in;
    in.edge_selector  = skill::chamfer_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in.chamfer_size_mm = 0.5;
    in.angle_deg       = 80.0;    // > 75

    auto r = skill::chamfer_edge::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundAngleError = false;
    for (const auto& f : r.findings)
        if (f.severity == "error" && f.message.find("angle") != std::string::npos) {
            foundAngleError = true; break;
        }
    EXPECT_TRUE(foundAngleError);
}

// ─── 4. Recognize: finds chamfer faces ────────────────────────────────────
TEST(SkillChamferEdge, RecognizeFindsChamferFaces)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::chamfer_edge::Input in;
    in.edge_selector  = skill::chamfer_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in.chamfer_size_mm = 1.5;
    in.angle_deg       = 45.0;
    auto out = skill::chamfer_edge::apply(*stock, in);

    auto candidates = skill::chamfer_edge::recognize(*out.workpiece);
    ASSERT_FALSE(candidates.empty());
    // At least one candidate should claim ≥ 2 bevel faces (rim chamfer).
    bool clusterFound = false;
    for (const auto& c : candidates) {
        if (c.matched_geometry.contains("bevel_face_ids") &&
            c.matched_geometry["bevel_face_ids"].size() >= 2) {
            clusterFound = true; break;
        }
    }
    EXPECT_TRUE(clusterFound);
}

// ─── 5. Round-trip: re-apply produces same face count + volume ───────────
TEST(SkillChamferEdge, RoundTripFaceCountStable)
{
    auto stock1 = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::chamfer_edge::Input in;
    in.edge_selector  = skill::chamfer_edge::EdgesAtZBand{ 10.0, 1e-3 };
    in.chamfer_size_mm = 0.8;
    in.angle_deg       = 45.0;
    auto out1 = skill::chamfer_edge::apply(*stock1, in);

    auto stock2 = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out2 = skill::chamfer_edge::apply(*stock2, in);

    EXPECT_EQ(out1.workpiece->faceCount(), out2.workpiece->faceCount());
    EXPECT_EQ(out1.workpiece->edgeCount(), out2.workpiece->edgeCount());
    EXPECT_NEAR(volumeOf(out1.workpiece->shape()),
                volumeOf(out2.workpiece->shape()), 1e-3);
}
