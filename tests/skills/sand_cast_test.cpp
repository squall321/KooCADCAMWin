// @lat: [[process/test-strategy#skill round-trip]]
//
// sand_cast — sand casting PROCESS skill (no geometric change).
//
// Cases (6):
//   1. Apply is a geometric no-op (volume / face count unchanged).
//   2. Signature records the casting metadata verbatim.
//   3. DFM-CAST-WALL rejects wall_thickness < 3.0 mm.
//   4. DFM-CAST-DRAFT rejects draft_angle < 1.0 deg.
//   5. DFM-CAST-MAT issues an INFO for unknown materials (still passes).
//   6. recognize() replays the casting metadata from history.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sand_cast.hpp"

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

// ─── 1. Apply: no geometric change ─────────────────────────────────────────
TEST(SkillSandCast, ApplyIsGeometricNoOp)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore     = volumeOf(stock->shape());
    const int    facesBefore   = stock->faceCount();

    skill::sand_cast::Input in;
    in.material          = "gray_iron";
    in.wall_thickness_mm = 4.0;
    in.draft_angle_deg   = 2.0;

    auto out = skill::sand_cast::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), facesBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("sand_cast"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_process_only"].get<bool>());
    EXPECT_EQ(out.signature.pattern["process_family"].get<std::string>(),
              std::string("casting"));
}

// ─── 2. Signature carries verbatim metadata ────────────────────────────────
TEST(SkillSandCast, SignatureRecordsCastingMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sand_cast::Input in;
    in.material          = "aluminum_a356";
    in.wall_thickness_mm = 5.5;
    in.draft_angle_deg   = 3.0;
    in.parting_line_z_mm = 5.0;

    auto out = skill::sand_cast::apply(*stock, in);

    EXPECT_EQ(out.signature.params["material"].get<std::string>(),
              std::string("aluminum_a356"));
    EXPECT_NEAR(out.signature.params["wall_thickness_mm"].get<double>(), 5.5, 1e-9);
    EXPECT_NEAR(out.signature.params["draft_angle_deg"].get<double>(),    3.0, 1e-9);
    EXPECT_NEAR(out.signature.params["parting_line_z_mm"].get<double>(),  5.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("sand_cast_mold"));
}

// ─── 3. DFM rejects sub-3mm wall ───────────────────────────────────────────
TEST(SkillSandCast, ValidateRejectsTooThinWall)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sand_cast::Input in;
    in.material          = "gray_iron";
    in.wall_thickness_mm = 2.0;        // < 3 mm
    in.draft_angle_deg   = 2.0;

    auto r = skill::sand_cast::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAST-WALL" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::sand_cast::apply(*stock, in), skill::SkillError);
}

// ─── 4. DFM rejects low draft (< 1 deg) ────────────────────────────────────
TEST(SkillSandCast, ValidateRejectsLowDraft)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sand_cast::Input in;
    in.material          = "gray_iron";
    in.wall_thickness_mm = 4.0;
    in.draft_angle_deg   = 0.5;        // < 1 deg

    auto r = skill::sand_cast::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAST-DRAFT" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::sand_cast::apply(*stock, in), skill::SkillError);
}

// ─── 5. Unknown material → INFO only, validate still passes ────────────────
TEST(SkillSandCast, UnknownMaterialIssuesInfoButPasses)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sand_cast::Input in;
    in.material          = "unobtainium";   // not in catalog
    in.wall_thickness_mm = 4.0;
    in.draft_angle_deg   = 2.0;

    auto r = skill::sand_cast::validate(*stock, in);
    EXPECT_TRUE(r.passed);                 // info, not error
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CAST-MAT" && f.severity == "info") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);

    // apply() succeeds since validate passed.
    auto out = skill::sand_cast::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("sand_cast"));
}

// ─── 6. Recognize replays the casting metadata from history ────────────────
TEST(SkillSandCast, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::sand_cast::Input in;
    in.material          = "ductile_iron";
    in.wall_thickness_mm = 6.0;
    in.draft_angle_deg   = 2.5;

    auto out   = skill::sand_cast::apply(*stock, in);
    auto cands = skill::sand_cast::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("sand_cast"));
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(),
              std::string("ductile_iron"));
    EXPECT_NEAR(cands[0].recovered_params["wall_thickness_mm"].get<double>(),
                6.0, 1e-9);

    // recognize on a feature-less stock returns nothing.
    auto bare = skill::createCuboidStock(50.0, 50.0, 10.0);
    EXPECT_TRUE(skill::sand_cast::recognize(*bare).empty());
}
