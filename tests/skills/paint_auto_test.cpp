// @lat: [[process/test-strategy#skill round-trip]]
//
// paint_auto — 5-case sweep covering identity-geometry synthesis,
// paint_color / coat_count / oven_temp DFM gates, signature, metadata
// replay, and the cycle-time formula (per-coat spray + bake).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/paint_auto.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ── 1. Apply: geometry COMPLETELY unchanged ──────────────────────────────
TEST(SkillPaintAuto, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::paint_auto::Input in;
    in.paint_color = "pearl_white";
    in.coat_count  = 4;
    in.oven_temp_c = 160.0;

    auto out = skill::paint_auto::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("paint_auto"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty color, bad coat count, bad oven temp rejected ──────────
TEST(SkillPaintAuto, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::paint_auto::Input bad;
    bad.paint_color = "";                       // invalid
    bad.coat_count  = 4;
    bad.oven_temp_c = 160.0;
    EXPECT_FALSE(skill::paint_auto::validate(*stock, bad).passed);
    EXPECT_THROW(skill::paint_auto::apply(*stock, bad), skill::SkillError);

    bad.paint_color = "red";
    bad.coat_count  = 0;                        // invalid
    EXPECT_FALSE(skill::paint_auto::validate(*stock, bad).passed);

    bad.coat_count  = 4;
    bad.oven_temp_c = 0.0;                      // invalid
    EXPECT_FALSE(skill::paint_auto::validate(*stock, bad).passed);

    bad.oven_temp_c = -30.0;                    // invalid
    EXPECT_FALSE(skill::paint_auto::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillPaintAuto, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::paint_auto::Input in;
    in.paint_color = "metallic_blue";
    in.coat_count  = 5;
    in.oven_temp_c = 175.0;

    auto out = skill::paint_auto::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("paint_auto"));
    EXPECT_EQ(out.signature.pattern["paint_color"].get<std::string>(),
              std::string("metallic_blue"));
    EXPECT_EQ(out.signature.pattern["coat_count"].get<int>(), 5);
    EXPECT_NEAR(out.signature.pattern["oven_temp_c"].get<double>(),
                175.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("paint_booth_oven"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPaintAuto, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::paint_auto::Input in;
    in.paint_color = "midnight_black";
    in.coat_count  = 3;
    in.oven_temp_c = 150.0;

    auto out = skill::paint_auto::apply(*stock, in);
    auto cands = skill::paint_auto::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["paint_color"].get<std::string>(),
              std::string("midnight_black"));
    EXPECT_EQ(cands[0].recovered_params["coat_count"].get<int>(), 3);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::paint_auto::recognize(raw).empty());
}

// ── 5. Cycle-time formula: coats * (spray + bake) ────────────────────────
TEST(SkillPaintAuto, CycleTimeMatchesFormula)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::paint_auto::Input in;
    in.paint_color = "silver";
    in.coat_count  = 4;
    in.oven_temp_c = 160.0;

    auto out = skill::paint_auto::apply(*stock, in);
    // 4 * (180 + 1200) = 4 * 1380 = 5520.
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 5520.0, 1e-9);

    // Out-of-band oven_temp_c emits info, still passes.
    skill::paint_auto::Input warm;
    warm.paint_color = "red";
    warm.coat_count  = 2;
    warm.oven_temp_c = 90.0;                    // below 120, info-only
    auto r = skill::paint_auto::validate(*stock, warm);
    EXPECT_TRUE(r.passed);
    bool sawInfo = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-PAINT-OVEN" && f.severity == "info") {
            sawInfo = true; break;
        }
    }
    EXPECT_TRUE(sawInfo);
}
