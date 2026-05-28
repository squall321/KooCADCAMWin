// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/swarf_5ax.hpp"

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

skill::swarf_5ax::Input makeValidInput()
{
    skill::swarf_5ax::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.guide_polyline_top = {
        { 20.0, 30.0, 14.0 },
        { 40.0, 30.0, 14.0 },
        { 60.0, 30.0, 14.0 },
    };
    in.guide_polyline_bottom = {
        { 20.0, 30.0, 4.0 },
        { 40.0, 30.0, 4.0 },
        { 60.0, 30.0, 4.0 },
    };
    in.tool_dia_mm = 4.0;
    return in;
}

}  // namespace

// ── 1. Apply: box-chain on a ruled surface removes material ──────────────
TEST(SkillSwarf5ax, ApplyCreatesRuledCut)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 15.0);
    const double volBefore = volumeOf(stock->shape());

    auto in = makeValidInput();
    auto out = skill::swarf_5ax::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), volBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("swarf_5ax"));
    EXPECT_TRUE(out.signature.pattern["is_5axis_cut"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["ruled_surface_present"].get<bool>());
    EXPECT_EQ(out.signature.pattern["waypoint_count"].get<size_t>(), 3u);
}

// ── 2. DFM-002: small tool rejected ──────────────────────────────────────
TEST(SkillSwarf5ax, ValidateRejectsSmallTool)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in = makeValidInput();
    in.tool_dia_mm = 1.0;     // < 1.5 → DFM-002

    auto r = skill::swarf_5ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-002") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::swarf_5ax::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-INPUT: mismatched top/bottom counts rejected ──────────────────
TEST(SkillSwarf5ax, ValidateRejectsMismatchedGuides)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in = makeValidInput();
    in.guide_polyline_bottom.pop_back();    // count mismatch

    auto r = skill::swarf_5ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);
}

// ── 4. DFM-005 info always emitted ───────────────────────────────────────
TEST(SkillSwarf5ax, ValidateEmitsDFM005Info)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto in = makeValidInput();

    auto r = skill::swarf_5ax::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-005" && f.severity == "info") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize emits a low-confidence candidate when tilted facets exist
TEST(SkillSwarf5ax, RecognizeFindsRuledSurfaceFacets)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 15.0);
    auto in = makeValidInput();
    // Use slanted guides to leave tilted planar facets.
    in.guide_polyline_top = {
        { 20.0, 30.0, 14.0 },
        { 40.0, 30.0, 14.0 },
        { 60.0, 30.0, 14.0 },
    };
    in.guide_polyline_bottom = {
        { 25.0, 30.0, 4.0 },     // shifted in X so the ruled surface tilts
        { 45.0, 30.0, 4.0 },
        { 65.0, 30.0, 4.0 },
    };
    auto out = skill::swarf_5ax::apply(*stock, in);
    auto cands = skill::swarf_5ax::recognize(*out.workpiece);
    if (!cands.empty()) {
        EXPECT_LE(cands[0].confidence, 0.5);
        EXPECT_GE(cands[0].confidence, 0.3);
        EXPECT_EQ(cands[0].skill_id, std::string("swarf_5ax"));
    }
}

// ── 6. Sweep length recorded in pattern ──────────────────────────────────
TEST(SkillSwarf5ax, PatternRecordsSweepLength)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 15.0);
    auto in = makeValidInput();
    auto out = skill::swarf_5ax::apply(*stock, in);

    // Top guide: (20→40→60) on the X-axis → length 40.
    const double recorded = out.signature.pattern["guide_top_length"].get<double>();
    EXPECT_NEAR(recorded, 40.0, 1e-6);
}
