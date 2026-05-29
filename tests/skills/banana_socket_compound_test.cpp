// @lat: [[process/test-strategy#skill compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/banana_socket_compound.hpp"

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

// 1. Apply produces non-null shape; volume delta within ±20 % of expected;
//    face count grew (entry + bore wall + step wall + groove + slits create new faces).
TEST(SkillBananaSocketCompound, ApplyCutsRealGeometry)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    const double volBefore = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();

    skill::banana_socket_compound::Input in;
    in.entry_face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm             = 20.0;
    in.position_y_mm             = 20.0;
    in.bore_dia_mm               = 4.0;
    in.bore_depth_mm             = 14.0;
    in.slit_width_mm             = 0.6;
    in.slit_depth_mm             = 10.0;
    in.retention_groove_dia_mm   = 4.6;
    in.retention_groove_depth_mm = 0.4;
    in.retention_groove_pos_mm   = 8.0;
    in.insulation_step_dia_mm    = 7.0;
    in.insulation_step_depth_mm  = 2.0;

    auto out = skill::banana_socket_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected removed volume:
    //   bore     : π·2² · 14 = 175.9
    //   step annulus  : π·(3.5²-2²) · 2 = 51.8
    //   groove annulus: π·(2.3²-2²) · 0.4 = 1.62
    //   slits  (approx, much harder — already mostly subsumed by bore;
    //           contribution outside the bore: 2 × (slitFullLen - bore_dia) ×
    //           slit_width × slit_depth ≈ 2 × (4.9 - 4.0) × 0.6 × 10 ≈ 10.8
    const double rBore = 2.0;
    const double rStep = 3.5;
    const double rGrv  = 2.3;
    const double slitOuterR = rGrv + 0.3;
    const double slitFullLen = slitOuterR * 2.0;
    const double approxRem =
        M_PI * rBore * rBore * 14.0
      + M_PI * (rStep*rStep - rBore*rBore) * 2.0
      + M_PI * (rGrv *rGrv  - rBore*rBore) * 0.4
      + 2.0 * (slitFullLen - 4.0) * 0.6 * 10.0;
    const double volAfter = volumeOf(out.workpiece->shape());
    const double diff = volBefore - volAfter;
    EXPECT_NEAR(diff, approxRem, approxRem * 0.20)
        << "banana socket should remove bore + step + groove + 2 slits";
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
}

// 2. Validate rejects bad input + apply throws SkillError.
TEST(SkillBananaSocketCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);

    skill::banana_socket_compound::Input bad;
    bad.entry_face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.position_x_mm             = 20.0;
    bad.position_y_mm             = 20.0;
    bad.bore_dia_mm               = 5.0;       // out of DIN 42220 [3.8, 4.2]
    bad.bore_depth_mm             = 14.0;
    bad.slit_width_mm             = 0.6;
    bad.slit_depth_mm             = 10.0;
    bad.retention_groove_dia_mm   = 5.6;
    bad.retention_groove_depth_mm = 0.4;
    bad.retention_groove_pos_mm   = 8.0;
    bad.insulation_step_dia_mm    = 8.0;
    bad.insulation_step_depth_mm  = 2.0;

    auto r = skill::banana_socket_compound::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool foundDin = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DIN42220") { foundDin = true; break; }
    EXPECT_TRUE(foundDin);
    EXPECT_THROW(skill::banana_socket_compound::apply(*stock, bad), skill::SkillError);
}

// 3. Signature: is_compound==true, subfeature_count >= 2, all params round-trip.
TEST(SkillBananaSocketCompound, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);

    skill::banana_socket_compound::Input in;
    in.entry_face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm             = 20.0;
    in.position_y_mm             = 20.0;
    in.bore_dia_mm               = 4.0;
    in.bore_depth_mm             = 14.0;
    in.slit_width_mm             = 0.6;
    in.slit_depth_mm             = 10.0;
    in.retention_groove_dia_mm   = 4.6;
    in.retention_groove_depth_mm = 0.4;
    in.retention_groove_pos_mm   = 8.0;
    in.insulation_step_dia_mm    = 7.0;
    in.insulation_step_depth_mm  = 2.0;

    auto out = skill::banana_socket_compound::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat["kind"].get<std::string>(), std::string("banana_socket_compound"));
    EXPECT_TRUE(pat["is_compound"].get<bool>());
    EXPECT_GE(pat["subfeature_count"].get<int>(), 2);
    EXPECT_EQ(pat["slit_count"].get<int>(), 4);

    const auto& p = out.signature.params;
    EXPECT_DOUBLE_EQ(p["bore_dia_mm"].get<double>(),               4.0);
    EXPECT_DOUBLE_EQ(p["bore_depth_mm"].get<double>(),             14.0);
    EXPECT_DOUBLE_EQ(p["retention_groove_dia_mm"].get<double>(),   4.6);
    EXPECT_DOUBLE_EQ(p["insulation_step_dia_mm"].get<double>(),    7.0);
    EXPECT_DOUBLE_EQ(p["insulation_step_depth_mm"].get<double>(),  2.0);
}

// 4. Recognize returns ≥1 candidate with confidence ≥ 0.5 (geometric) and 1.0 (metadata).
TEST(SkillBananaSocketCompound, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::banana_socket_compound::Input in;
    in.entry_face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm             = 20.0;
    in.position_y_mm             = 20.0;
    in.bore_dia_mm               = 4.0;
    in.bore_depth_mm             = 14.0;
    in.slit_width_mm             = 0.6;
    in.slit_depth_mm             = 10.0;
    in.retention_groove_dia_mm   = 4.6;
    in.retention_groove_depth_mm = 0.4;
    in.retention_groove_pos_mm   = 8.0;
    in.insulation_step_dia_mm    = 7.0;
    in.insulation_step_depth_mm  = 2.0;

    auto out = skill::banana_socket_compound::apply(*stock, in);
    auto cands = skill::banana_socket_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    // At least one of them should hit 1.0 (metadata) — must be present since
    // apply registered the feature.
    bool hasMeta = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-6) { hasMeta = true; break; }
    }
    EXPECT_TRUE(hasMeta);
}

// 5. Feature-specific: bore_dia in DIN 42220 4 mm class.
TEST(SkillBananaSocketCompound, FeatureSpecific_DinClass)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);
    skill::banana_socket_compound::Input in;
    in.entry_face                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm             = 20.0;
    in.position_y_mm             = 20.0;
    in.bore_dia_mm               = 4.0;
    in.bore_depth_mm             = 14.0;
    in.slit_width_mm             = 0.6;
    in.slit_depth_mm             = 10.0;
    in.retention_groove_dia_mm   = 4.6;
    in.retention_groove_depth_mm = 0.4;
    in.retention_groove_pos_mm   = 8.0;
    in.insulation_step_dia_mm    = 7.0;
    in.insulation_step_depth_mm  = 2.0;

    auto out = skill::banana_socket_compound::apply(*stock, in);
    const double recoveredBore = out.signature.params["bore_dia_mm"].get<double>();
    EXPECT_GE(recoveredBore, 3.8);
    EXPECT_LE(recoveredBore, 4.2);
}
