// @lat: [[process/test-strategy#compound cooling_plate_channel_serpentine]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cooling_plate_channel_serpentine.hpp"

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

skill::cooling_plate_channel_serpentine::Input goodInput(const skill::Workpiece& wp) {
    skill::cooling_plate_channel_serpentine::Input in;
    in.face_id            = findTopFaceId(wp);
    in.channel_width_mm   = 6.0;
    in.channel_depth_mm   = 3.0;
    in.channel_pitch_mm   = 12.0;
    in.channel_count      = 4;
    in.straight_length_mm = 80.0;
    in.inlet_x_mm         = 20.0;
    in.inlet_y_mm         = 10.0;
    return in;
}
}  // namespace

// ─── 1. Apply cuts serpentine channel ────────────────────────────────────
TEST(SkillCoolingSerpentine, ApplyCutsSerpentineChannel)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 10.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::cooling_plate_channel_serpentine::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0);  // material removed
}

// ─── 2. Validate rejects bad width/depth aspect ──────────────────────────
TEST(SkillCoolingSerpentine, ValidateRejectsBadAspect)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 10.0);
    auto in    = goodInput(*stock);
    in.channel_width_mm = 1.0;   // ratio = 1/3 = 0.33 < 0.4
    in.channel_depth_mm = 3.0;

    auto r = skill::cooling_plate_channel_serpentine::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ASPECT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects rib wall too thin ───────────────────────────────
TEST(SkillCoolingSerpentine, ValidateRejectsThinRibWall)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 10.0);
    auto in    = goodInput(*stock);
    in.channel_pitch_mm = in.channel_width_mm + 1.0;  // < width + 1.5

    auto r = skill::cooling_plate_channel_serpentine::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RIB-WALL") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records serpentine + channel_count ─────────────────────
TEST(SkillCoolingSerpentine, SignatureRecordsSerpentineAndCount)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 10.0);
    auto in    = goodInput(*stock);
    auto out   = skill::cooling_plate_channel_serpentine::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("cooling_plate_channel_serpentine"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("is_serpentine").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("ev_feature_type").get<std::string>(),
              std::string("cooling_plate_channel"));
    EXPECT_EQ(out.signature.pattern.at("channel_count").get<int>(),
              in.channel_count);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCoolingSerpentine, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 120.0, 10.0);
    auto in    = goodInput(*stock);
    auto out   = skill::cooling_plate_channel_serpentine::apply(*stock, in);

    auto cands = skill::cooling_plate_channel_serpentine::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("channel_count").get<int>(),
              in.channel_count);
}
