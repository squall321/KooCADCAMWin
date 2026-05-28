// @lat: [[process/test-strategy#skill round-trip]]
//
// riveted_joint — through-hole + small head bump on each side.
//
// Cases (6):
//   1. Apply cuts the bore and fuses two head bumps.
//   2. Signature carries rivet/head metadata.
//   3. DFM rejects sub-1.6 mm rivet diameter.
//   4. DFM rejects head_dia smaller than 1.5 × rivet_dia.
//   5. DFM rejects excessive head_height.
//   6. Recognize replays history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/riveted_joint.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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

// ─── 1. Apply produces through-hole plus two head bumps ──────────────────
TEST(SkillRivetedJoint, ApplyMakesBoreAndHeads)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::riveted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.rivet_dia_mm  = 4.0;
    in.head_dia_mm   = 8.0;     // 2 × rivet_dia
    in.head_height_mm = 1.0;
    in.material      = "aluminum";

    auto out = skill::riveted_joint::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Net volume = -hole + 2 × head bumps
    const double hole = M_PI * 2.0 * 2.0 * 10.0;
    const double head = M_PI * 4.0 * 4.0 * 1.0;
    const double expectedDelta = 2.0 * head - hole;       // (+) if heads outweigh hole
    const double actualDelta   = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_NEAR(actualDelta, expectedDelta, std::abs(expectedDelta) * 0.10 + 1.0);

    EXPECT_EQ(out.signature.skill_id, std::string("riveted_joint"));
}

// ─── 2. Signature carries metadata ───────────────────────────────────────
TEST(SkillRivetedJoint, SignatureCarriesMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::riveted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.rivet_dia_mm   = 3.0;
    in.head_dia_mm    = 5.0;
    in.head_height_mm = 0.8;

    auto out = skill::riveted_joint::apply(*stock, in);
    EXPECT_NEAR(out.signature.params["rivet_dia_mm"].get<double>(),   3.0, 1e-9);
    EXPECT_NEAR(out.signature.params["head_dia_mm"].get<double>(),    5.0, 1e-9);
    EXPECT_NEAR(out.signature.params["head_height_mm"].get<double>(), 0.8, 1e-9);
    EXPECT_EQ(out.signature.pattern["cylindrical_face_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("rivet_set"));
}

// ─── 3. DFM rejects rivet_dia < 1.6 ──────────────────────────────────────
TEST(SkillRivetedJoint, ValidateRejectsTinyShank)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::riveted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.rivet_dia_mm   = 1.0;       // < 1.6
    in.head_dia_mm    = 2.5;
    in.head_height_mm = 0.5;

    auto r = skill::riveted_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RJ-DIA") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
    EXPECT_THROW(skill::riveted_joint::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects small head ───────────────────────────────────────────
TEST(SkillRivetedJoint, ValidateRejectsSmallHead)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::riveted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.rivet_dia_mm   = 4.0;
    in.head_dia_mm    = 5.0;       // < 1.5 × 4.0
    in.head_height_mm = 1.0;

    auto r = skill::riveted_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RJ-HEAD-RATIO") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 5. DFM rejects over-tall head ───────────────────────────────────────
TEST(SkillRivetedJoint, ValidateRejectsTallHead)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::riveted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 25.0;
    in.position_y_mm = 25.0;
    in.rivet_dia_mm   = 4.0;
    in.head_dia_mm    = 8.0;
    in.head_height_mm = 5.0;       // > 8.0 × 0.5

    auto r = skill::riveted_joint::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RJ-HEAD-H") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 6. Recognize replays history ────────────────────────────────────────
TEST(SkillRivetedJoint, RecognizeReplaysFromHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::riveted_joint::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 30.0;
    in.rivet_dia_mm   = 3.0;
    in.head_dia_mm    = 5.5;
    in.head_height_mm = 0.7;
    in.material       = "stainless";

    auto out = skill::riveted_joint::apply(*stock, in);
    auto cands = skill::riveted_joint::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    const auto& c = cands.front();
    EXPECT_NEAR(c.confidence, 1.0, 1e-9);
    EXPECT_NEAR(c.recovered_params["rivet_dia_mm"].get<double>(),   3.0, 1e-6);
    EXPECT_NEAR(c.recovered_params["head_dia_mm"].get<double>(),    5.5, 1e-6);
    EXPECT_NEAR(c.recovered_params["head_height_mm"].get<double>(), 0.7, 1e-6);
    EXPECT_EQ(c.recovered_params["material"].get<std::string>(), std::string("stainless"));
}
