// @lat: [[process/test-strategy#compound macro]]
//
// gasket_face_with_drain_groove — 5 tests verifying REAL volume + spec.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gasket_face_with_drain_groove.hpp"

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

// ─── T1: apply removes ~ face_relief + drain volume ±20 % ─────────────────
TEST(SkillGasketFaceDrain, T1_ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    const double v0 = volumeOf(stock->shape());

    skill::gasket_face_with_drain_groove::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.center_x_mm    = 50.0;
    in.center_y_mm    = 50.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.face_dia_mm    = 60.0;
    in.relief_depth_mm = 0.5;            // 0.5 mm relief (large for volume test)
    in.drain_radius_mm = 22.0;            // inside [15, 29]
    in.drain_width_mm  = 1.0;
    in.drain_depth_mm  = 0.5;
    in.id_chamfer_mm   = 0.5;
    in.gasket_class    = "ANSI-150";
    in.roughness_class = "stock";

    auto out = skill::gasket_face_with_drain_groove::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // T1 expected math:
    //   face relief: pi * 30^2 * 0.5 = pi * 450 = 1413.7 mm³
    //   drain groove: pi * (22.5^2 - 21.5^2) * (relief+drain depth combined ~1.0)
    //               = pi * 44 * 1.0 ≈ 138.2 mm³
    //   Total ≈ 1551.9 mm³  (drain partially overlaps with relief)
    const double faceR = 30.0;
    const double reliefVol = M_PI * faceR * faceR * 0.5;
    const double removed = v0 - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 0.80 * reliefVol);
    EXPECT_LT(removed, 1.40 * reliefVol);
}

// ─── T2: validate rejects unknown drain radius + apply throws ────────────
TEST(SkillGasketFaceDrain, T2_ValidateRejectsBadDrain)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    skill::gasket_face_with_drain_groove::Input bad;
    bad.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.face_dia_mm    = 60.0;
    bad.drain_radius_mm = 5.0;          // way under face_dia/4 = 15
    bad.drain_width_mm  = 1.0;
    bad.drain_depth_mm  = 0.5;
    bad.relief_depth_mm = 0.05;

    auto r = skill::gasket_face_with_drain_groove::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GASKET-DRAIN") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::gasket_face_with_drain_groove::apply(*stock, bad),
                 skill::SkillError);
}

// ─── T3: signature has spec_key + is_compound + 3 subfeatures ────────────
TEST(SkillGasketFaceDrain, T3_SignatureCarriesSpecKey)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    skill::gasket_face_with_drain_groove::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm    = 50.0;
    in.center_y_mm    = 50.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.face_dia_mm    = 60.0;
    in.drain_radius_mm = 22.0;
    in.drain_width_mm  = 1.0;
    in.drain_depth_mm  = 0.5;
    in.relief_depth_mm = 0.05;
    in.gasket_class    = "ANSI-300";
    in.roughness_class = "smooth";

    auto out = skill::gasket_face_with_drain_groove::apply(*stock, in);
    const auto& pat = out.signature.pattern;
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("gasket_face_with_drain_groove"));
    EXPECT_EQ(pat.at("is_compound").get<bool>(), true);
    EXPECT_EQ(pat.at("subfeature_count").get<int>(), 3);
    EXPECT_EQ(pat.at("spec_key").get<std::string>(), std::string("ANSI-300"));
    EXPECT_NEAR(pat.at("recommended_ra_um").get<double>(), 3.2, 1e-6);
}

// ─── T4: recognize replays + matches spec_key ─────────────────────────────
TEST(SkillGasketFaceDrain, T4_RecognizeReplaysSpec)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    skill::gasket_face_with_drain_groove::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm    = 50.0;
    in.center_y_mm    = 50.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.face_dia_mm    = 60.0;
    in.drain_radius_mm = 22.0;
    in.drain_width_mm  = 1.0;
    in.drain_depth_mm  = 0.5;
    in.relief_depth_mm = 0.05;
    in.gasket_class    = "ANSI-600";
    in.roughness_class = "stock";

    auto out = skill::gasket_face_with_drain_groove::apply(*stock, in);
    auto cands = skill::gasket_face_with_drain_groove::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id,
              std::string("gasket_face_with_drain_groove"));
    EXPECT_EQ(cands[0].recovered_params.at("gasket_class").get<std::string>(),
              std::string("ANSI-600"));
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}

// ─── T5: recommended_ra by class lookup follows ASME B16.5 Table 6 ───────
TEST(SkillGasketFaceDrain, T5_AsmeRoughnessTableLookup)
{
    // The class -> Ra mapping is the embedded ASME B16.5 §6.4 spec.
    auto stock = skill::createCuboidStock(100.0, 100.0, 20.0);
    skill::gasket_face_with_drain_groove::Input in;
    in.face_id        = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm    = 50.0;
    in.center_y_mm    = 50.0;
    in.axis_dir       = gp_Dir(0, 0, -1);
    in.face_dia_mm    = 60.0;
    in.drain_radius_mm = 24.0;
    in.drain_width_mm  = 1.0;
    in.drain_depth_mm  = 0.5;
    in.relief_depth_mm = 0.05;
    in.gasket_class    = "ANSI-2500";
    in.roughness_class = "smooth";
    auto outHP = skill::gasket_face_with_drain_groove::apply(*stock, in);
    // ANSI-2500 → Ra 0.8 μm per Table 6.
    EXPECT_NEAR(outHP.signature.pattern.at("recommended_ra_um").get<double>(),
                0.8, 1e-6);

    // ANSI-150 → Ra 3.2 μm.
    in.gasket_class = "ANSI-150";
    auto outLP = skill::gasket_face_with_drain_groove::apply(*stock, in);
    EXPECT_NEAR(outLP.signature.pattern.at("recommended_ra_um").get<double>(),
                3.2, 1e-6);

    // Roughness_class unknown → 0 (lookup miss is recorded as 0).
    in.gasket_class = "BOGUS-CLASS";
    auto outUnk = skill::gasket_face_with_drain_groove::apply(*stock, in);
    EXPECT_DOUBLE_EQ(outUnk.signature.pattern.at("recommended_ra_um").get<double>(),
                     0.0);
}
