// @lat: [[process/test-strategy#skill round-trip]]
//
// loft_surface — skin through ≥ 2 closed cross-section wires.
//
// Cases (6):
//   1. ApplyProducesNonNull shape.
//   2. ValidateRejectsBadInput — only one section provided.
//   3. SignatureRecordsKind — pattern.kind == "loft_surface".
//   4. RecognizeMetadataReplay — history replay confidence 1.0.
//   5. ValidateRejectsOpenSection — section with < 3 distinct vertices.
//   6. ApplyAddsSolidVolume — solid loft adds material between sections.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/loft_surface.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

std::vector<gp_Pnt> squareSection(double cx, double cy, double cz, double half)
{
    return {
        gp_Pnt(cx - half, cy - half, cz),
        gp_Pnt(cx + half, cy - half, cz),
        gp_Pnt(cx + half, cy + half, cz),
        gp_Pnt(cx - half, cy + half, cz),
    };
}

}  // namespace

// ─── 1. ApplyProducesNonNull shape ────────────────────────────────────────
TEST(SkillLoftSurface, ApplyProducesNonNull)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::loft_surface::Input in;
    in.sections = {
        squareSection(20.0, 15.0, 15.0, 6.0),
        squareSection(20.0, 15.0, 25.0, 4.0),
    };
    in.ruled = false;

    auto out = skill::loft_surface::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput (1 section) ───────────────────────────────
TEST(SkillLoftSurface, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::loft_surface::Input in;
    in.sections = {
        squareSection(20.0, 15.0, 15.0, 6.0),
    };

    auto r = skill::loft_surface::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::loft_surface::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind ──────────────────────────────────────────────
TEST(SkillLoftSurface, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::loft_surface::Input in;
    in.sections = {
        squareSection(20.0, 15.0, 15.0, 6.0),
        squareSection(20.0, 15.0, 25.0, 4.0),
    };

    auto out = skill::loft_surface::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("loft_surface"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("loft_surface"));
    EXPECT_EQ(out.signature.pattern["section_count"].get<int>(), 2);
}

// ─── 4. RecognizeMetadataReplay (confidence 1.0) ──────────────────────────
TEST(SkillLoftSurface, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::loft_surface::Input in;
    in.sections = {
        squareSection(20.0, 15.0, 15.0, 6.0),
        squareSection(20.0, 15.0, 20.0, 5.0),
        squareSection(20.0, 15.0, 25.0, 4.0),
    };
    in.ruled = true;

    auto out = skill::loft_surface::apply(*stock, in);
    auto cands = skill::loft_surface::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("loft_surface"));
    EXPECT_DOUBLE_EQ(c.confidence, 1.0);
    EXPECT_EQ(c.recovered_params["section_count"].get<int>(), 3);
    EXPECT_TRUE(c.recovered_params["ruled"].get<bool>());
}

// ─── 5. ValidateRejectsOpenSection ────────────────────────────────────────
TEST(SkillLoftSurface, ValidateRejectsOpenSection)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::loft_surface::Input in;
    in.sections = {
        // Only 2 distinct vertices — can't form a closed polygon.
        { gp_Pnt(0, 0, 5), gp_Pnt(10, 0, 5) },
        squareSection(20.0, 15.0, 15.0, 4.0),
    };

    auto r = skill::loft_surface::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LOFT-CLOSED") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 6. ApplyAddsSolidVolume ──────────────────────────────────────────────
TEST(SkillLoftSurface, ApplyAddsSolidVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    // Build two parallel square sections, both well above the stock so the
    // loft adds a clear volume.
    skill::loft_surface::Input in;
    in.sections = {
        squareSection(20.0, 15.0, 15.0, 5.0),
        squareSection(20.0, 15.0, 25.0, 3.0),
    };
    in.ruled = true;       // linear blend → predictable volume

    auto out = skill::loft_surface::apply(*stock, in);
    const double newVol = volumeOf(out.workpiece->shape());
    EXPECT_GT(newVol, stockVol) << "ruled loft between 2 sections should add a solid";
}
