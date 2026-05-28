// @lat: [[process/test-strategy#skill round-trip]]
//
// tig_weld skill — 6-case sweep covering synthesis, DFM, recognize, tooling
// metadata, and gas/material discrimination.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tig_weld.hpp"

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

// ─── 1. Apply: bead fuses onto top face (additive volume) ─────────────────
TEST(SkillTigWeld, ApplyAddsBead)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::tig_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 4.0;
    in.bead_height_mm = 1.0;
    in.material = "stainless";
    in.gas      = "argon";

    auto out = skill::tig_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double r = 2.0;
    const double L = 40.0;
    const double approxAdded = (M_PI * r * r + L * 4.0) * 1.0;
    const double added = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_GT(added, approxAdded * 0.5);
    EXPECT_LT(added, approxAdded * 1.5);

    EXPECT_EQ(out.signature.skill_id, std::string("tig_weld"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("tig_torch"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("tungsten"));
    EXPECT_EQ(out.signature.pattern["kind"], "tig_weld");
}

// ─── 2. DFM: bead outside [1, 8] rejected ────────────────────────────────
TEST(SkillTigWeld, ValidateRejectsBadWidth)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::tig_weld::Input narrow;
    narrow.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    narrow.start_x_mm = 20.0; narrow.start_y_mm = 20.0;
    narrow.end_x_mm   = 60.0; narrow.end_y_mm   = 20.0;
    narrow.bead_width_mm  = 0.5;     // < 1.0
    narrow.bead_height_mm = 0.2;
    auto r1 = skill::tig_weld::validate(*stock, narrow);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::tig_weld::apply(*stock, narrow), skill::SkillError);

    skill::tig_weld::Input wide = narrow;
    wide.bead_width_mm  = 10.0;      // > 8.0
    wide.bead_height_mm = 1.0;
    auto r2 = skill::tig_weld::validate(*stock, wide);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. DFM: tall bead (height > 0.5 × width) rejected ───────────────────
TEST(SkillTigWeld, ValidateRejectsTallBead)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::tig_weld::Input tall;
    tall.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tall.start_x_mm = 20.0; tall.start_y_mm = 20.0;
    tall.end_x_mm   = 60.0; tall.end_y_mm   = 20.0;
    tall.bead_width_mm  = 3.0;
    tall.bead_height_mm = 2.0;     // 2 / 3 > 0.5

    auto r = skill::tig_weld::validate(*stock, tall);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TIG-RATIO") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Recognize via metadata (full-confidence replay) ───────────────────
TEST(SkillTigWeld, RecognizeViaMetadata)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::tig_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 5.0;
    in.bead_height_mm = 1.0;
    in.material = "aluminum";
    in.gas      = "argon_helium";

    auto out = skill::tig_weld::apply(*stock, in);
    auto cands = skill::tig_weld::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_NEAR(cands[0].recovered_params["bead_width_mm"].get<double>(), 5.0, 1e-3);
    EXPECT_EQ(cands[0].recovered_params["gas"].get<std::string>(), "argon_helium");
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(), "aluminum");
}

// ─── 5. Recognize anonymous workpiece → no candidates (geom ambiguous) ────
TEST(SkillTigWeld, RecognizeAnonymousReturnsEmpty)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::tig_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 4.0;
    in.bead_height_mm = 1.0;
    auto out = skill::tig_weld::apply(*stock, in);

    // Recreate workpiece WITHOUT features — geom-only path.  We refuse to
    // claim a bead is TIG without metadata to avoid double-classification.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands = skill::tig_weld::recognize(raw);
    EXPECT_TRUE(cands.empty());
}

// ─── 6. Tooling metadata: negative stock removal + tig_torch ──────────────
TEST(SkillTigWeld, ToolingMetaRecordsAdditive)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    skill::tig_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 70.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 4.0;
    in.bead_height_mm = 1.0;
    in.material = "steel";
    in.gas      = "argon";

    auto out = skill::tig_weld::apply(*stock, in);
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("tig_torch"));
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
    EXPECT_EQ(out.signature.tooling.extra["process"], "tig");
    EXPECT_EQ(out.signature.tooling.extra["gas"], "argon");
}
