// @lat: [[process/test-strategy#compound feature]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/t_slot_table_groove.hpp"

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

// ─── 1. ApplyProducesValidShapeWithMultipleSubFeatures ──────────────────
TEST(SkillTSlotTableGroove, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 30.0);

    skill::t_slot_table_groove::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.start_x_mm = 10.0;
    in.start_y_mm = 50.0;
    in.end_x_mm   = 190.0;
    in.end_y_mm   = 50.0;
    in.axis_dir   = gp_Dir(0, 0, -1);
    in.top_w_mm   = 14.0;
    in.top_d_mm   = 8.0;
    in.bot_w_mm   = 24.0;
    in.bot_d_mm   = 10.0;

    auto out = skill::t_slot_table_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume removed ~= (top_w × top_d + bot_w × bot_d) × slotLen
    //   = (14×8 + 24×10) × 180 = (112 + 240) × 180 = 63360 mm³
    const double approx = (14.0*8.0 + 24.0*10.0) * 180.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.10);

    // Face count: the T-cross-section adds at minimum 2 horizontal step
    // faces + 4 vertical walls + 1 floor.
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount() + 4);
}

// ─── 2. ValidateRejectsBadInput ─────────────────────────────────────────
TEST(SkillTSlotTableGroove, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 30.0);

    skill::t_slot_table_groove::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0;
    in.start_y_mm = 50.0;
    in.end_x_mm   = 190.0;
    in.end_y_mm   = 50.0;
    in.top_w_mm   = 14.0;
    in.top_d_mm   = 8.0;
    in.bot_w_mm   = 10.0;  // ≤ top_w → DFM-TSLOT
    in.bot_d_mm   = 10.0;

    auto r = skill::t_slot_table_groove::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TSLOT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::t_slot_table_groove::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. SignatureRecordsCompoundKind ────────────────────────────────────
TEST(SkillTSlotTableGroove, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 30.0);
    skill::t_slot_table_groove::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0;
    in.start_y_mm = 50.0;
    in.end_x_mm   = 190.0;
    in.end_y_mm   = 50.0;
    in.top_w_mm   = 14.0;
    in.top_d_mm   = 8.0;
    in.bot_w_mm   = 24.0;
    in.bot_d_mm   = 10.0;
    auto out = skill::t_slot_table_groove::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("t_slot_table_groove"));
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_NEAR(pat.at("total_depth_mm").get<double>(),
                in.top_d_mm + in.bot_d_mm, 1e-6);
}

// ─── 4. RecognizeMetadataReplay ─────────────────────────────────────────
TEST(SkillTSlotTableGroove, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 30.0);
    skill::t_slot_table_groove::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0;
    in.start_y_mm = 50.0;
    in.end_x_mm   = 190.0;
    in.end_y_mm   = 50.0;
    in.top_w_mm   = 14.0;
    in.top_d_mm   = 8.0;
    in.bot_w_mm   = 24.0;
    in.bot_d_mm   = 10.0;
    auto out = skill::t_slot_table_groove::apply(*stock, in);

    auto cands = skill::t_slot_table_groove::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("top_w_mm").get<double>(),
                14.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params.at("bot_w_mm").get<double>(),
                24.0, 1e-6);
}

// ─── 5. specific: top_w < bot_w (T-slot geometry constraint) ─────────────
TEST(SkillTSlotTableGroove, TopWidthLessThanBottomWidth)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 30.0);
    skill::t_slot_table_groove::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.start_x_mm = 10.0;
    in.start_y_mm = 50.0;
    in.end_x_mm   = 190.0;
    in.end_y_mm   = 50.0;
    in.top_w_mm   = 14.0;
    in.top_d_mm   = 8.0;
    in.bot_w_mm   = 24.0;
    in.bot_d_mm   = 10.0;
    auto out = skill::t_slot_table_groove::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    // The T-shape invariant: top_w < bot_w.
    EXPECT_LT(pat.at("top_w_mm").get<double>(),
              pat.at("bot_w_mm").get<double>());

    // DIN 650 ratio bot/top ∈ [1.5, 3.5].
    const double ratio = pat.at("bot_to_top_ratio").get<double>();
    EXPECT_GE(ratio, 1.5);
    EXPECT_LE(ratio, 3.5);
    EXPECT_NEAR(ratio, 24.0 / 14.0, 1e-6);
}
