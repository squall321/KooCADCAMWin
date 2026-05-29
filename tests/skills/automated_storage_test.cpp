// @lat: [[process/test-strategy#skill round-trip]]
//
// automated_storage — 5-case sweep covering identity-geometry synthesis,
// bin id / height / load_class DFM gates, metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/automated_storage.hpp"

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

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillAutomatedStorage, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::automated_storage::Input in;
    in.bin_id           = "B-07-Lv02";
    in.storage_height_m = 8.0;
    in.load_class       = "medium";

    auto out = skill::automated_storage::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("automated_storage"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty bin / zero height rejected ─────────────────────────────
TEST(SkillAutomatedStorage, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::automated_storage::Input bad;
    bad.bin_id           = "";          // invalid
    bad.storage_height_m = 5.0;
    auto r = skill::automated_storage::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::automated_storage::apply(*stock, bad), skill::SkillError);

    bad.bin_id           = "A-01-Lv01";
    bad.storage_height_m = 0.0;          // invalid
    EXPECT_FALSE(skill::automated_storage::validate(*stock, bad).passed);

    bad.storage_height_m = -2.0;
    EXPECT_FALSE(skill::automated_storage::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + bin_id/height/class ──────────────────────
TEST(SkillAutomatedStorage, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::automated_storage::Input in;
    in.bin_id           = "C-22-Lv05";
    in.storage_height_m = 12.5;
    in.load_class       = "heavy";

    auto out = skill::automated_storage::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("automated_storage"));
    EXPECT_EQ(out.signature.pattern["bin_id"].get<std::string>(),
              std::string("C-22-Lv05"));
    EXPECT_NEAR(out.signature.pattern["storage_height_m"].get<double>(),
                12.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["load_class"].get<std::string>(),
              std::string("heavy"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("asrs_shuttle"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillAutomatedStorage, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::automated_storage::Input in;
    in.bin_id           = "D-99-Lv09";
    in.storage_height_m = 18.0;
    in.load_class       = "light";

    auto out = skill::automated_storage::apply(*stock, in);
    auto cands = skill::automated_storage::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["bin_id"].get<std::string>(),
              std::string("D-99-Lv09"));
    EXPECT_NEAR(cands[0].recovered_params["storage_height_m"].get<double>(),
                18.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::automated_storage::recognize(raw).empty());
}

// ── 5. est_cycle_time_s scales with storage height ───────────────────────
TEST(SkillAutomatedStorage, CycleTimeScalesWithHeight)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::automated_storage::Input low;
    low.bin_id           = "A-01-Lv01";
    low.storage_height_m = 2.0;
    low.load_class       = "light";
    auto outLow = skill::automated_storage::apply(*stock, low);

    skill::automated_storage::Input hi;
    hi.bin_id           = "A-01-Lv09";
    hi.storage_height_m = 20.0;
    hi.load_class       = "light";
    auto outHi = skill::automated_storage::apply(*stock, hi);

    EXPECT_GT(outHi.signature.tooling.est_cycle_time_s,
              outLow.signature.tooling.est_cycle_time_s);
    EXPECT_NEAR(outLow.signature.tooling.est_cycle_time_s, 6.0, 1e-9);
    EXPECT_NEAR(outHi.signature.tooling.est_cycle_time_s,  24.0, 1e-9);
}
