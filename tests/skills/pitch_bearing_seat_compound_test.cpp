// @lat: [[process/test-strategy#wind pitch_bearing_seat_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pitch_bearing_seat_compound.hpp"

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

skill::pitch_bearing_seat_compound::Input goodInput(const skill::Workpiece& wp) {
    skill::pitch_bearing_seat_compound::Input in;
    in.face_id              = findTopFaceId(wp);
    in.center_x_mm          = 300.0;
    in.center_y_mm          = 300.0;
    in.bearing_inner_dia_mm = 300.0;
    in.bearing_outer_dia_mm = 400.0;
    in.bearing_thickness_mm = 40.0;
    in.bolt_count           = 12;   // > 4
    in.bolt_pcd_mm          = 460.0;  // > outer_dia
    in.bolt_thread_size_key = "M16";
    return in;
}
}  // namespace

// ─── 1. Apply cuts annular seat + bolt circle ───────────────────────────
TEST(SkillPitchBearingSeat, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 60.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::pitch_bearing_seat_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(),
              1 + in.bolt_count);
}

// ─── 2. Validate rejects bolt_count <= 4 ────────────────────────────────
TEST(SkillPitchBearingSeat, ValidateRejectsTooFewBolts)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 60.0);
    auto in    = goodInput(*stock);
    in.bolt_count = 4;

    auto r = skill::pitch_bearing_seat_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BOLT-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 3. Validate rejects unknown M-thread key ───────────────────────────
TEST(SkillPitchBearingSeat, ValidateRejectsUnknownThreadKey)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 60.0);
    auto in    = goodInput(*stock);
    in.bolt_thread_size_key = "M99";  // not in _iso_thread_table

    auto r = skill::pitch_bearing_seat_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Validate rejects thickness/outer_dia ratio outside 0.02..0.30 ──
TEST(SkillPitchBearingSeat, ValidateRejectsBadThicknessRatio)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 60.0);
    auto in    = goodInput(*stock);
    in.bearing_thickness_mm = 200.0;  // ratio 200/400 = 0.5 > 0.30

    auto r = skill::pitch_bearing_seat_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BEARING-RATIO") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 5. Signature + Recognize round-trip ────────────────────────────────
TEST(SkillPitchBearingSeat, SignatureAndRecognize)
{
    auto stock = skill::createCuboidStock(600.0, 600.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::pitch_bearing_seat_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("pitch_bearing_seat_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("wind_feature_type").get<std::string>(),
              std::string("pitch_bearing_seat"));
    EXPECT_EQ(out.signature.pattern.at("bolt_count").get<int>(), in.bolt_count);

    auto cands = skill::pitch_bearing_seat_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("bolt_thread_size_key")
                  .get<std::string>(), in.bolt_thread_size_key);
}
