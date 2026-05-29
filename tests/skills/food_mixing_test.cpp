// @lat: [[process/test-strategy#skill round-trip]]
//
// food_mixing — bulk food mixing metadata stamp.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. ShortMixTimeEmitsInfo

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/food_mixing.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillFoodMixing, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::food_mixing::Input in;
    in.mixer_type   = "ribbon";
    in.batch_kg     = 100.0;
    in.mix_time_min = 15.0;

    auto out = skill::food_mixing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillFoodMixing, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 3.0);

    // Unknown mixer type.
    skill::food_mixing::Input badMixer;
    badMixer.mixer_type   = "blender_3000";
    badMixer.batch_kg     = 50.0;
    badMixer.mix_time_min = 10.0;
    {
        auto r = skill::food_mixing::validate(*stock, badMixer);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::food_mixing::apply(*stock, badMixer),
                 skill::SkillError);

    // Zero batch.
    skill::food_mixing::Input zeroBatch;
    zeroBatch.mixer_type   = "ribbon";
    zeroBatch.batch_kg     = 0.0;
    zeroBatch.mix_time_min = 10.0;
    {
        auto r = skill::food_mixing::validate(*stock, zeroBatch);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-MIX-BATCH"));
    }

    // Time too long.
    skill::food_mixing::Input longTime;
    longTime.mixer_type   = "paddle";
    longTime.batch_kg     = 200.0;
    longTime.mix_time_min = 300.0;     // > 240
    {
        auto r = skill::food_mixing::validate(*stock, longTime);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-MIX-TIME"));
    }
}

// ─── 3. Signature records kind + key params ────────────────────────────────
TEST(SkillFoodMixing, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::food_mixing::Input in;
    in.mixer_type   = "planetary";
    in.batch_kg     = 75.0;
    in.mix_time_min = 20.0;

    auto out = skill::food_mixing::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("food_mixing"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("food_mixing"));
    EXPECT_EQ(out.signature.pattern["mixer_type"].get<std::string>(),
              std::string("planetary"));
    EXPECT_NEAR(out.signature.pattern["batch_kg"].get<double>(),
                75.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["mix_time_min"].get<double>(),
                20.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.pattern["process_family"].get<std::string>(),
              std::string("food_processing"));
}

// ─── 4. Recognize metadata replay ──────────────────────────────────────────
TEST(SkillFoodMixing, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 3.0);

    skill::food_mixing::Input in;
    in.mixer_type   = "high_shear";
    in.batch_kg     = 500.0;
    in.mix_time_min = 8.0;

    auto out   = skill::food_mixing::apply(*stock, in);
    auto cands = skill::food_mixing::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["mixer_type"].get<std::string>(),
              std::string("high_shear"));
    EXPECT_NEAR(cands[0].recovered_params["batch_kg"].get<double>(),
                500.0, 1e-9);

    // Stripped of metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::food_mixing::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: short mix time emits info, still proceeds ────────────────
TEST(SkillFoodMixing, ShortMixTimeEmitsInfo)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 2.0);

    skill::food_mixing::Input in;
    in.mixer_type   = "double_cone";
    in.batch_kg     = 25.0;
    in.mix_time_min = 0.5;      // < 1 → info

    auto r = skill::food_mixing::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MIX-TIME-SHORT"));

    auto out = skill::food_mixing::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("food_mixer_double_cone"));
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 30.0, 1e-9);
}
