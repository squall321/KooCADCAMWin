// @lat: [[process/test-strategy#skill round-trip]]
//
// thread_mill_external skill tests (slice — external thread variant).
//   1. Apply on a cylindrical stock cuts a small volume (real helical sweep).
//   2. DFM-EXT-TURNS catches insufficient turns (length / pitch < 3).
//   3. DFM-002 catches sub-min tool diameter.
//   4. DFM-EXT-MIN catches too-small cylinder.
//   5. Recognize replays signature.
//   6. Pattern carries is_external=true and turns count.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/thread_mill_external.hpp"

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
}  // namespace

// ── 1. Apply on cylindrical stock cuts an external thread ────────────────
TEST(SkillThreadMillExternal, ApplyCutsExternalThread)
{
    // Start from a Ø 8 mm round bar — large enough for M6 external thread.
    auto stock = skill::createCylindricalStock(8.0, 20.0);

    const double volBefore = volumeOf(stock->shape());

    skill::thread_mill_external::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 6.0;     // 6 turns
    in.tool_dia_mm      = 1.5;

    auto out = skill::thread_mill_external::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // External cut → workpiece volume must NOT INCREASE.
    EXPECT_LE(volumeOf(out.workpiece->shape()), volBefore + 1e-6);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("thread_mill_external"));
    EXPECT_TRUE(out.signature.pattern.at("is_external").get<bool>());

    const std::string geom = out.signature.pattern["geometry"].get<std::string>();
    EXPECT_TRUE(geom == "helical_swept" || geom == "metadata_only")
        << "unexpected geometry tag: " << geom;

    // Turns = thread_length / pitch = 6.0
    EXPECT_NEAR(out.signature.pattern["turns"].get<double>(), 6.0, 1e-6);
}

// ── 2. DFM-EXT-TURNS catches too-few turns ───────────────────────────────
TEST(SkillThreadMillExternal, ValidateRejectsTooFewTurns)
{
    auto stock = skill::createCylindricalStock(8.0, 20.0);

    skill::thread_mill_external::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 2.0;     // only 2 turns — DFM threshold is 3
    in.tool_dia_mm      = 1.5;

    auto r = skill::thread_mill_external::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EXT-TURNS") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::thread_mill_external::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-002 catches sub-min tool diameter ─────────────────────────────
TEST(SkillThreadMillExternal, ValidateRejectsSubMinTool)
{
    auto stock = skill::createCylindricalStock(8.0, 20.0);

    skill::thread_mill_external::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 6.0;
    in.tool_dia_mm      = 0.5;     // < 0.8 mm

    auto r = skill::thread_mill_external::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") found = true;
    EXPECT_TRUE(found);
}

// ── 4. DFM-EXT-MIN catches too-small cylinder ────────────────────────────
TEST(SkillThreadMillExternal, ValidateRejectsTooSmallCylinder)
{
    // 2 mm cylinder < M3 minor (2.5 mm).
    auto stock = skill::createCylindricalStock(2.0, 10.0);

    skill::thread_mill_external::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M2";
    in.pitch_mm         = 0.4;
    in.thread_length_mm = 3.0;
    in.tool_dia_mm      = 1.0;

    auto r = skill::thread_mill_external::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EXT-MIN") found = true;
    EXPECT_TRUE(found);
}

// ── 5. Recognize replays signature ───────────────────────────────────────
TEST(SkillThreadMillExternal, RecognizeReplaysSignature)
{
    auto stock = skill::createCylindricalStock(10.0, 25.0);

    skill::thread_mill_external::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M8";
    in.pitch_mm         = 1.25;
    in.thread_length_mm = 10.0;
    in.tool_dia_mm      = 2.0;

    auto out   = skill::thread_mill_external::apply(*stock, in);
    auto cands = skill::thread_mill_external::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("thread_mill_external"));
    EXPECT_EQ(cands[0].recovered_params["thread_size"], "M8");
    EXPECT_NEAR(cands[0].recovered_params["pitch_mm"].get<double>(), 1.25, 1e-6);
    EXPECT_NEAR(cands[0].confidence, 0.5, 1e-6);
}

// ── 6. Tooling metadata carries is_external + turns ──────────────────────
TEST(SkillThreadMillExternal, ToolingCarriesExternalMetadata)
{
    auto stock = skill::createCylindricalStock(8.0, 20.0);

    skill::thread_mill_external::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 6.0;
    in.tool_dia_mm      = 1.5;

    auto out = skill::thread_mill_external::apply(*stock, in);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("thread_mill_external"));
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 1.5, 1e-6);
    EXPECT_EQ(out.signature.tooling.flute_count, 3);
    EXPECT_TRUE(out.signature.tooling.extra.at("is_external").get<bool>());
    EXPECT_NEAR(out.signature.tooling.extra.at("turns").get<double>(), 6.0, 1e-6);
}
