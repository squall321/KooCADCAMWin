// @lat: [[process/test-strategy#skill round-trip]]
//
// electrochemical_etch skill — stencil-based shallow etching tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/electrochemical_etch.hpp"

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

// ── 1. Apply etches glyph-shaped depressions ─────────────────────────────
TEST(SkillElectrochemicalEtch, ApplyRemovesGlyphVolume)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::electrochemical_etch::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.direction_xy  = gp_Dir(1, 0, 0);
    in.text          = "X9";
    in.font_size_mm  = 3.0;
    in.etch_depth_um = 60.0;

    auto out = skill::electrochemical_etch::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.0);
    EXPECT_LT(removed, 5.0);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("electrochemical_etch"));
    EXPECT_EQ(out.signature.pattern["char_count"].get<size_t>(), 2u);
    EXPECT_NEAR(out.signature.pattern["etch_depth_um"].get<double>(), 60.0, 1e-6);
}

// ── 2. DFM-ETCH-DEPTH rejects out-of-range depth ─────────────────────────
TEST(SkillElectrochemicalEtch, ValidateRejectsBadDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::electrochemical_etch::Input tooShallow;
    tooShallow.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooShallow.text         = "K";
    tooShallow.font_size_mm = 2.0;
    tooShallow.etch_depth_um = 5.0;  // < 10
    auto r1 = skill::electrochemical_etch::validate(*stock, tooShallow);
    EXPECT_FALSE(r1.passed);
    bool found1 = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-ETCH-DEPTH") { found1 = true; break; }
    EXPECT_TRUE(found1);
    EXPECT_THROW(skill::electrochemical_etch::apply(*stock, tooShallow),
                 skill::SkillError);

    skill::electrochemical_etch::Input tooDeep = tooShallow;
    tooDeep.etch_depth_um = 250.0;  // > 200
    auto r2 = skill::electrochemical_etch::validate(*stock, tooDeep);
    EXPECT_FALSE(r2.passed);
}

// ── 3. DFM-ETCH-FONT enforces 1 mm stencil minimum ───────────────────────
TEST(SkillElectrochemicalEtch, ValidateRejectsSmallFont)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::electrochemical_etch::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text          = "X";
    in.font_size_mm  = 0.8;  // < 1.0
    in.etch_depth_um = 50.0;

    auto r = skill::electrochemical_etch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ETCH-FONT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. Empty text rejected ───────────────────────────────────────────────
TEST(SkillElectrochemicalEtch, ValidateRejectsEmptyText)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::electrochemical_etch::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.text          = "";
    in.font_size_mm  = 2.0;
    in.etch_depth_um = 50.0;

    auto r = skill::electrochemical_etch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ── 5. Recognize replays metadata ────────────────────────────────────────
TEST(SkillElectrochemicalEtch, RecognizeReplaysMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::electrochemical_etch::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 8.0;
    in.position_y_mm = 25.0;
    in.text          = "LOT42";
    in.font_size_mm  = 2.5;
    in.etch_depth_um = 80.0;

    auto out = skill::electrochemical_etch::apply(*stock, in);
    auto cands = skill::electrochemical_etch::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands[0].skill_id, std::string("electrochemical_etch"));
    EXPECT_EQ(cands[0].recovered_params["text"], "LOT42");
    EXPECT_NEAR(cands[0].recovered_params["font_size_mm"].get<double>(), 2.5, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["etch_depth_um"].get<double>(), 80.0, 1e-6);
    EXPECT_GT(cands[0].confidence, 0.9);
}

// ── 6. Tooling metadata uses electrochemical_etcher ──────────────────────
TEST(SkillElectrochemicalEtch, ToolingProcessTag)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::electrochemical_etch::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0;
    in.position_y_mm = 25.0;
    in.text          = "SN";
    in.font_size_mm  = 2.0;
    in.etch_depth_um = 40.0;

    auto out = skill::electrochemical_etch::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("electrochemical_etcher"));
    EXPECT_EQ(out.signature.tooling.extra["process"].get<std::string>(),
              std::string("electrochemical_etch"));
}
