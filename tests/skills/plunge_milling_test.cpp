// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/plunge_milling.hpp"

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

// ── 1. Apply: plunge cut removes π r² × depth ────────────────────────────
TEST(SkillPlungeMilling, ApplyCreatesPlungeCut)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);

    skill::plunge_milling::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.dia_mm       = 8.0;
    in.depth_mm     = 5.0;

    auto out = skill::plunge_milling::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double expected = M_PI * 4.0 * 4.0 * 5.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, expected, expected * 0.05);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("plunge_milling"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("plunge_milling"));
}

// ── 2. DFM-002: dia < 0.8 rejected ───────────────────────────────────────
TEST(SkillPlungeMilling, ValidateRejectsSubMin)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::plunge_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.dia_mm = 0.5; in.depth_mm = 2.0;

    auto r = skill::plunge_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::plunge_milling::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-PLUNGE-AR: depth/dia > 3 rejected ─────────────────────────────
TEST(SkillPlungeMilling, ValidateRejectsHighAspectRatio)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 50.0);
    skill::plunge_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 30.0;
    in.dia_mm = 3.0; in.depth_mm = 20.0;     // depth/dia = 6.67 → error

    auto r = skill::plunge_milling::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PLUNGE-AR") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. Signature marks cutting mode as plunge_only ───────────────────────
TEST(SkillPlungeMilling, SignatureMarksPlungeOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::plunge_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0; in.position_y_mm = 25.0;
    in.dia_mm = 6.0; in.depth_mm = 4.0;

    auto out = skill::plunge_milling::apply(*stock, in);
    EXPECT_TRUE(out.signature.tooling.extra.contains("cutting_mode"));
    EXPECT_EQ(out.signature.tooling.extra["cutting_mode"].get<std::string>(),
              std::string("plunge_only"));
}

// ── 5. Recognize finds the plunge cut (aspect ratio in range) ────────────
TEST(SkillPlungeMilling, RecognizeFindsPlunge)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    skill::plunge_milling::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 30.0;
    in.dia_mm = 8.0; in.depth_mm = 5.0;    // depth/dia = 0.625

    auto out = skill::plunge_milling::apply(*stock, in);
    auto cands = skill::plunge_milling::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].recovered_params["dia_mm"].get<double>(),   8.0, 0.1);
    EXPECT_NEAR(cands[0].recovered_params["depth_mm"].get<double>(), 5.0, 0.1);
}
