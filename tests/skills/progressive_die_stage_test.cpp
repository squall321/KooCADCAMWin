// @lat: [[process/test-strategy#skill round-trip]]
//
// progressive_die_stage skill — metadata-only stage marker.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/progressive_die_stage.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: geometry is COMPLETELY unchanged ────────────────────────
TEST(SkillProgressiveDieStage, ApplyChangesGeometryOrPreserves)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::progressive_die_stage::Input in;
    in.stage_index             = 3;
    in.station_position        = "pierce";
    in.material_strip_pitch_mm = 25.0;

    auto out = skill::progressive_die_stage::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. Validate rejects bad input: stage_index < 1 ────────────────────
TEST(SkillProgressiveDieStage, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::progressive_die_stage::Input in;
    in.stage_index             = 0;       // invalid (must be ≥ 1)
    in.station_position        = "blank";
    in.material_strip_pitch_mm = 25.0;

    auto r = skill::progressive_die_stage::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::progressive_die_stage::apply(*stock, in), skill::SkillError);

    // Negative pitch.
    skill::progressive_die_stage::Input in2;
    in2.stage_index             = 1;
    in2.station_position        = "blank";
    in2.material_strip_pitch_mm = -5.0;
    auto r2 = skill::progressive_die_stage::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
}

// ── 3. Signature records kind + stage_index + station + pitch ─────────
TEST(SkillProgressiveDieStage, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::progressive_die_stage::Input in;
    in.stage_index             = 5;
    in.station_position        = "bend";
    in.material_strip_pitch_mm = 30.0;

    auto out = skill::progressive_die_stage::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("progressive_die_stage"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("progressive_die_stage"));
    EXPECT_EQ(out.signature.pattern["stage_index"].get<int>(), 5);
    EXPECT_EQ(out.signature.pattern["station_position"].get<std::string>(),
              std::string("bend"));
    EXPECT_NEAR(out.signature.pattern["material_strip_pitch_mm"].get<double>(),
                30.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("progressive_die_station"));
}

// ── 4. Recognize via metadata replay ──────────────────────────────────
TEST(SkillProgressiveDieStage, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::progressive_die_stage::Input in;
    in.stage_index             = 7;
    in.station_position        = "trim";
    in.material_strip_pitch_mm = 40.0;

    auto out = skill::progressive_die_stage::apply(*stock, in);
    auto cands = skill::progressive_die_stage::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["stage_index"].get<int>(), 7);
    EXPECT_EQ(cands[0].recovered_params["station_position"].get<std::string>(),
              std::string("trim"));

    // Stripped of metadata → recognize empty (no geometric trace).
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::progressive_die_stage::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ── 5. Specific: feature chain accumulates across multiple stages ─────
TEST(SkillProgressiveDieStage, AccumulatesAcrossStages)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::progressive_die_stage::Input s1;
    s1.stage_index             = 1;
    s1.station_position        = "blank";
    s1.material_strip_pitch_mm = 25.0;
    auto out1 = skill::progressive_die_stage::apply(*stock, s1);

    skill::progressive_die_stage::Input s2;
    s2.stage_index             = 2;
    s2.station_position        = "pierce";
    s2.material_strip_pitch_mm = 25.0;
    auto out2 = skill::progressive_die_stage::apply(*out1.workpiece, s2);

    EXPECT_EQ(out2.workpiece->features().size(), 2u);
    EXPECT_EQ(out2.workpiece->features()[0].pattern["stage_index"].get<int>(), 1);
    EXPECT_EQ(out2.workpiece->features()[1].pattern["stage_index"].get<int>(), 2);
}
