// @lat: [[process/test-strategy#wind generator_stator_slot_pattern]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/generator_stator_slot_pattern.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFaceId(const skill::Workpiece& wp) {
    int best = -1;
    double bestZ = -1e9;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const auto n = wp.faceNormal(i);
        if (n.Z() < 0.9) continue;
        const auto c = wp.faceCenter(i);
        if (c.Z() > bestZ) { bestZ = c.Z(); best = i; }
    }
    return best;
}

skill::generator_stator_slot_pattern::Input goodInput(const skill::Workpiece& wp) {
    skill::generator_stator_slot_pattern::Input in;
    in.face_id               = findTopFaceId(wp);
    in.center_x_mm           = 200.0;
    in.center_y_mm           = 200.0;
    in.stator_inner_dia_mm   = 200.0;
    in.stator_outer_dia_mm   = 300.0;
    in.slot_count            = 36;       // smaller of {36, 48, 72}
    in.slot_width_mm         = 8.0;      // < π·200/36 = 17.45
    in.slot_depth_mm         = 30.0;     // < wall(50) - 2
    in.slot_opening_width_mm = 3.0;
    in.stator_thickness_mm   = 40.0;
    return in;
}
}  // namespace

// ─── 1. Apply removes material with slot_count subfeatures ──────────────
TEST(SkillGeneratorStatorSlot, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(400.0, 400.0, 60.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::generator_stator_slot_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              in.slot_count);
}

// ─── 2. Validate rejects slot_count <= 4 ────────────────────────────────
TEST(SkillGeneratorStatorSlot, ValidateRejectsTooFewSlots)
{
    auto stock = skill::createCuboidStock(400.0, 400.0, 60.0);
    auto in    = goodInput(*stock);
    in.slot_count = 4;

    auto r = skill::generator_stator_slot_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate enforces slot_width < π·stator_inner_dia / slot_count ─
TEST(SkillGeneratorStatorSlot, ValidateRejectsOverlappingSlots)
{
    auto stock = skill::createCuboidStock(400.0, 400.0, 60.0);
    auto in    = goodInput(*stock);
    // π·200/36 ≈ 17.45 mm. Force slot_width above that.
    in.slot_width_mm = 25.0;

    auto r = skill::generator_stator_slot_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT-FIT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::generator_stator_slot_pattern::apply(*stock, in),
                 skill::SkillError);
}

// ─── 4. Validate rejects slot_opening >= slot_width ─────────────────────
TEST(SkillGeneratorStatorSlot, ValidateRejectsBadOpening)
{
    auto stock = skill::createCuboidStock(400.0, 400.0, 60.0);
    auto in    = goodInput(*stock);
    in.slot_opening_width_mm = in.slot_width_mm;

    auto r = skill::generator_stator_slot_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-SLOT-OPEN") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Signature + Recognize round-trip ────────────────────────────────
TEST(SkillGeneratorStatorSlot, SignatureAndRecognize)
{
    auto stock = skill::createCuboidStock(400.0, 400.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::generator_stator_slot_pattern::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("generator_stator_slot_pattern"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("wind_feature_type").get<std::string>(),
              std::string("generator_stator_slot_pattern"));
    EXPECT_EQ(out.signature.pattern.at("slot_count").get<int>(), in.slot_count);
    EXPECT_NEAR(out.signature.pattern.at("slot_width_mm").get<double>(),
                in.slot_width_mm, 1e-6);

    auto cands = skill::generator_stator_slot_pattern::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    // At least one candidate must be metadata replay with confidence 1.0.
    bool replayFound = false;
    for (const auto& c : cands) {
        if (c.confidence >= 0.99 &&
            c.recovered_params.contains("slot_count") &&
            c.recovered_params.at("slot_count").get<int>() == in.slot_count) {
            replayFound = true;
            break;
        }
    }
    EXPECT_TRUE(replayFound);
}
