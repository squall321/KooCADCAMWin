// @lat: [[process/test-strategy#skill round-trip]]
//
// ultrasonic_machine — USM abrasive-slurry pocket round-trip tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ultrasonic_machine.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply removes a square pocket ─────────────────────────────────────
TEST(SkillUltrasonicMachine, ApplyRemovesMaterialSquare)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "glass_borosilicate");

    skill::ultrasonic_machine::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.shape         = "square";
    in.length_mm     = 15.0;
    in.width_mm      = 10.0;
    in.depth_mm      = 4.0;

    auto out = skill::ultrasonic_machine::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = 15.0 * 10.0 * 4.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);
}

// ── 2. Validate rejects unknown shape ────────────────────────────────────
TEST(SkillUltrasonicMachine, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "alumina");

    skill::ultrasonic_machine::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.shape         = "hexagon";   // unsupported
    in.length_mm     = 8.0;
    in.width_mm      = 8.0;
    in.depth_mm      = 2.0;

    auto r = skill::ultrasonic_machine::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::ultrasonic_machine::apply(*stock, in), skill::SkillError);
}

// ── 3. Signature records kind + tool_type ────────────────────────────────
TEST(SkillUltrasonicMachine, SignatureRecordsKindAndToolType)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "fused_silica");

    skill::ultrasonic_machine::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.shape         = "round";
    in.diameter_mm   = 8.0;
    in.depth_mm      = 3.0;

    auto out = skill::ultrasonic_machine::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("ultrasonic_machine"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ultrasonic_machine"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("usm_horn"));
    EXPECT_TRUE(out.signature.pattern["material_brittle"].get<bool>());
}

// ── 4. Recognize replays metadata ────────────────────────────────────────
TEST(SkillUltrasonicMachine, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "zirconia");

    skill::ultrasonic_machine::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.shape         = "square";
    in.length_mm     = 10.0;
    in.width_mm      = 6.0;
    in.depth_mm      = 2.0;

    auto out = skill::ultrasonic_machine::apply(*stock, in);
    auto cands = skill::ultrasonic_machine::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_EQ(cands[0].recovered_params["shape"].get<std::string>(),
              std::string("square"));
}

// ── 5. Specific: non-brittle material emits the USM-material info finding
TEST(SkillUltrasonicMachine, NonBrittleMaterialEmitsInfo)
{
    // aluminum_6061 — ductile, NOT in the brittle catalog.
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::ultrasonic_machine::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 20.0;
    in.shape         = "round";
    in.diameter_mm   = 6.0;
    in.depth_mm      = 2.0;

    auto r = skill::ultrasonic_machine::validate(*stock, in);
    EXPECT_TRUE(r.passed);     // info findings don't fail validation
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-USM-MATERIAL" && f.severity == "info") {
            found = true; break;
        }
    EXPECT_TRUE(found);
}
