// @lat: [[process/test-strategy#compound pv_junction_box_pottable_pocket]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pv_junction_box_pottable_pocket.hpp"

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

skill::pv_junction_box_pottable_pocket::Input goodInput(const skill::Workpiece& wp) {
    skill::pv_junction_box_pottable_pocket::Input in;
    in.face_id               = findTopFaceId(wp);
    in.center_x_mm           = 75.0;
    in.center_y_mm           = 50.0;
    in.pocket_w_mm           = 132.0;
    in.pocket_h_mm           = 40.0;
    in.pocket_depth_mm       = 12.0;
    in.gland_count           = 4;
    in.gland_thread_size_key = "M16";
    return in;
}
}  // namespace

// ─── 1. Apply: cuts pocket + glands ──────────────────────────────────────
TEST(SkillPvJBoxPocket, ApplyCutsPocketAndGlands)
{
    auto stock = skill::createCuboidStock(150.0, 100.0, 25.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::pv_junction_box_pottable_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

// ─── 2. Validate rejects unknown thread key ──────────────────────────────
TEST(SkillPvJBoxPocket, ValidateRejectsUnknownThreadKey)
{
    auto stock = skill::createCuboidStock(150.0, 100.0, 25.0);
    auto in    = goodInput(*stock);
    in.gland_thread_size_key = "M99";

    auto r = skill::pv_junction_box_pottable_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD-KEY") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects gland_count not in {3,4} ────────────────────────
TEST(SkillPvJBoxPocket, ValidateRejectsBadGlandCount)
{
    auto stock = skill::createCuboidStock(150.0, 100.0, 25.0);
    auto in    = goodInput(*stock);
    in.gland_count = 5;

    auto r = skill::pv_junction_box_pottable_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PV-GLAND-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + gland count ─────────────────────────
TEST(SkillPvJBoxPocket, SignatureRecordsCompoundAndGlandCount)
{
    auto stock = skill::createCuboidStock(150.0, 100.0, 25.0);
    auto in    = goodInput(*stock);
    auto out   = skill::pv_junction_box_pottable_pocket::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("pv_junction_box_pottable_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("solar_feature_type").get<std::string>(),
              std::string("pv_junction_box_pocket"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              1 + in.gland_count);
    EXPECT_EQ(out.signature.pattern.at("gland_thread_size_key").get<std::string>(),
              std::string("M16"));
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillPvJBoxPocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(150.0, 100.0, 25.0);
    auto in    = goodInput(*stock);
    auto out   = skill::pv_junction_box_pottable_pocket::apply(*stock, in);

    auto cands = skill::pv_junction_box_pottable_pocket::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("gland_count").get<int>(),
              in.gland_count);
}
