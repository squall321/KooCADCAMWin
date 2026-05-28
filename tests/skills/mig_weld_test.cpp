// @lat: [[process/test-strategy#skill round-trip]]
//
// mig_weld skill — 6-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mig_weld.hpp"

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

// ─── 1. Apply: bead fuses onto top, additive volume ───────────────────────
TEST(SkillMigWeld, ApplyAddsWiderBead)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::mig_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 8.0;   // wider than TIG
    in.bead_height_mm = 2.0;   // taller than TIG
    in.wire_dia_mm    = 1.2;

    auto out = skill::mig_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double r = 4.0;
    const double L = 40.0;
    const double approxAdded = (M_PI * r * r + L * 8.0) * 2.0;
    const double added = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_GT(added, approxAdded * 0.5);
    EXPECT_LT(added, approxAdded * 1.5);

    EXPECT_EQ(out.signature.skill_id, std::string("mig_weld"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("mig_torch"));
    EXPECT_EQ(out.signature.pattern["kind"], "mig_weld");
}

// ─── 2. DFM: bead width outside [2, 12] rejected ──────────────────────────
TEST(SkillMigWeld, ValidateRejectsBadWidth)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::mig_weld::Input narrow;
    narrow.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    narrow.start_x_mm = 20.0; narrow.start_y_mm = 20.0;
    narrow.end_x_mm   = 60.0; narrow.end_y_mm   = 20.0;
    narrow.bead_width_mm  = 1.0;     // < 2.0
    narrow.bead_height_mm = 0.4;
    auto r1 = skill::mig_weld::validate(*stock, narrow);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::mig_weld::apply(*stock, narrow), skill::SkillError);

    skill::mig_weld::Input wide = narrow;
    wide.bead_width_mm  = 15.0;      // > 12.0
    wide.bead_height_mm = 2.0;
    auto r2 = skill::mig_weld::validate(*stock, wide);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. DFM: wire diameter outside [0.6, 1.6] rejected ────────────────────
TEST(SkillMigWeld, ValidateRejectsBadWireDia)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::mig_weld::Input thinWire;
    thinWire.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    thinWire.start_x_mm = 20.0; thinWire.start_y_mm = 20.0;
    thinWire.end_x_mm   = 60.0; thinWire.end_y_mm   = 20.0;
    thinWire.bead_width_mm  = 6.0;
    thinWire.bead_height_mm = 1.5;
    thinWire.wire_dia_mm    = 0.4;   // < 0.6
    auto r1 = skill::mig_weld::validate(*stock, thinWire);
    EXPECT_FALSE(r1.passed);
    bool found1 = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-MIG-WIRE") { found1 = true; break; }
    EXPECT_TRUE(found1);

    skill::mig_weld::Input thickWire = thinWire;
    thickWire.wire_dia_mm = 2.0;     // > 1.6
    auto r2 = skill::mig_weld::validate(*stock, thickWire);
    EXPECT_FALSE(r2.passed);
}

// ─── 4. Recognize via metadata ────────────────────────────────────────────
TEST(SkillMigWeld, RecognizeViaMetadata)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::mig_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 20.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 60.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 6.0;
    in.bead_height_mm = 2.0;
    in.wire_dia_mm    = 1.0;
    in.wire_feed_mm_per_s = 100.0;

    auto out = skill::mig_weld::apply(*stock, in);
    auto cands = skill::mig_weld::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_NEAR(cands[0].recovered_params["bead_width_mm"].get<double>(), 6.0, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["wire_dia_mm"].get<double>(), 1.0, 1e-3);
    EXPECT_NEAR(cands[0].recovered_params["wire_feed_mm_per_s"].get<double>(),
                100.0, 1e-3);
}

// ─── 5. Tooling metadata: consumable wire recorded as steel_wire ──────────
TEST(SkillMigWeld, ToolingMetaWire)
{
    auto stock = skill::createCuboidStock(80.0, 40.0, 10.0);

    skill::mig_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0; in.start_y_mm = 20.0;
    in.end_x_mm   = 70.0; in.end_y_mm   = 20.0;
    in.bead_width_mm  = 5.0;
    in.bead_height_mm = 1.5;
    in.wire_dia_mm    = 1.0;

    auto out = skill::mig_weld::apply(*stock, in);
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("steel_wire"));
    EXPECT_NEAR(
        out.signature.tooling.extra["wire_dia_mm"].get<double>(), 1.0, 1e-3);
}

// ─── 6. Diagonal seam — orientation math sanity ───────────────────────────
TEST(SkillMigWeld, DiagonalSeam)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::mig_weld::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 15.0; in.start_y_mm = 15.0;
    in.end_x_mm   = 45.0; in.end_y_mm   = 45.0;
    in.bead_width_mm  = 6.0;
    in.bead_height_mm = 1.5;
    in.wire_dia_mm    = 1.0;

    auto out = skill::mig_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), volBefore);
}
