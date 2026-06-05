// @lat: [[process/test-strategy#compound mounting_clamp_rail_slot]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/mounting_clamp_rail_slot.hpp"

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

skill::mounting_clamp_rail_slot::Input goodInput(const skill::Workpiece& wp) {
    skill::mounting_clamp_rail_slot::Input in;
    in.face_id               = findTopFaceId(wp);
    in.rail_axis_origin_x_mm = 5.0;
    in.rail_axis_origin_y_mm = 20.0;
    in.rail_length_mm        = 90.0;
    in.slot_top_width_mm     = 8.0;
    in.slot_throat_width_mm  = 5.0;
    in.slot_depth_mm         = 12.0;
    return in;
}
}  // namespace

// ─── 1. Apply: removes material along the rail ───────────────────────────
TEST(SkillMountingClampRail, ApplyCutsTSlotProfile)
{
    auto stock = skill::createCuboidStock(100.0, 40.0, 20.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::mounting_clamp_rail_slot::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects throat >= top width ─────────────────────────────
TEST(SkillMountingClampRail, ValidateRejectsThroatNotNarrower)
{
    auto stock = skill::createCuboidStock(100.0, 40.0, 20.0);
    auto in    = goodInput(*stock);
    in.slot_throat_width_mm = in.slot_top_width_mm;   // equal, invalid

    auto r = skill::mounting_clamp_rail_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TSLOT-THROAT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects negative rail_length ────────────────────────────
TEST(SkillMountingClampRail, ValidateRejectsZeroLength)
{
    auto stock = skill::createCuboidStock(100.0, 40.0, 20.0);
    auto in    = goodInput(*stock);
    in.rail_length_mm = 0.0;

    auto r = skill::mounting_clamp_rail_slot::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature records 2 subfeatures (bay + throat) ───────────────────
TEST(SkillMountingClampRail, SignatureRecordsTwoSubfeatures)
{
    auto stock = skill::createCuboidStock(100.0, 40.0, 20.0);
    auto in    = goodInput(*stock);
    auto out   = skill::mounting_clamp_rail_slot::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("mounting_clamp_rail_slot"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("solar_feature_type").get<std::string>(),
              std::string("t_slot_rail"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillMountingClampRail, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 40.0, 20.0);
    auto in    = goodInput(*stock);
    auto out   = skill::mounting_clamp_rail_slot::apply(*stock, in);

    auto cands = skill::mounting_clamp_rail_slot::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("slot_top_width_mm").get<double>(),
              in.slot_top_width_mm);
}
