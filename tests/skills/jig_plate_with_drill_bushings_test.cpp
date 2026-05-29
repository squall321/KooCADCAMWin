// @lat: [[process/test-strategy#compound jig_plate_with_drill_bushings]]
//
// Verifies REAL multi-cut geometry: pilot + seat + flange-seat + chamfer
// per site × N sites, plus DFM gates from DIN 179 / ASME Y14.5.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/jig_plate_with_drill_bushings.hpp"

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

// ─── 1. apply: real volume delta + face count grew ───────────────────────
TEST(SkillJigPlateWithDrillBushings, ApplyRealGeometryVolumeAndFaces)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 12.0);
    const int   stockFaces = stock->faceCount();
    const double stockVol  = volumeOf(stock->shape());

    skill::jig_plate_with_drill_bushings::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.bushing_id_mm    = 5.0;
    in.bushing_od_mm    = 10.0;
    in.flange_od_mm     = 13.0;
    in.flange_depth_mm  = 3.0;
    in.chamfer_mm       = 0.5;
    in.sites = {
        { 20.0, 30.0 },
        { 60.0, 30.0 },
    };

    auto out = skill::jig_plate_with_drill_bushings::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Per site volume:
    //   pilot Ø5 × 12 mm  = π·6.25·12  ≈ 235.6
    //   seat annulus (Ø10 − Ø5) × 3 mm = π·(25 − 6.25)·3 ≈ 176.7
    //   flange recess annulus (Ø13 − Ø10) × (chamfer + 0.5 = 1.0 mm)
    //                                   = π·(42.25 − 25)·1.0 ≈ 54.2
    //   chamfer band tiny (we ignore)
    const double rPilot = 2.5, rSeat = 5.0, rFlange = 6.5;
    const double perSite =
        M_PI * rPilot * rPilot * 12.0 +
        M_PI * (rSeat * rSeat - rPilot * rPilot) * 3.0 +
        M_PI * (rFlange * rFlange - rSeat * rSeat) * 1.0;
    const double approx  = perSite * 2.0;
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.15);

    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── 2. validate rejects bad input + apply throws ────────────────────────
TEST(SkillJigPlateWithDrillBushings, ValidateRejectsBadInputAndApplyThrows)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 12.0);

    // (a) empty sites
    {
        skill::jig_plate_with_drill_bushings::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.bushing_id_mm = 5.0; in.bushing_od_mm = 10.0;
        in.flange_od_mm  = 13.0; in.flange_depth_mm = 3.0;
        auto r = skill::jig_plate_with_drill_bushings::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::jig_plate_with_drill_bushings::apply(*stock, in),
                     skill::SkillError);
    }
    // (b) flange OD ≤ bushing OD
    {
        skill::jig_plate_with_drill_bushings::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.bushing_id_mm = 5.0; in.bushing_od_mm = 10.0;
        in.flange_od_mm  = 10.0; in.flange_depth_mm = 3.0;
        in.sites = { { 25.0, 25.0 } };
        auto r = skill::jig_plate_with_drill_bushings::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-FLANGE") { found = true; break; }
        EXPECT_TRUE(found);
    }
    // (c) thin wall
    {
        skill::jig_plate_with_drill_bushings::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.bushing_id_mm = 5.0; in.bushing_od_mm = 6.0;   // wall 0.5 < 1.0
        in.flange_od_mm  = 9.0; in.flange_depth_mm = 3.0;
        in.sites = { { 25.0, 25.0 } };
        auto r = skill::jig_plate_with_drill_bushings::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-BUSHING-WALL") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature: is_compound, subfeature_count, params round-trip ─────
TEST(SkillJigPlateWithDrillBushings, SignatureCompoundAndRoundTrip)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 10.0);

    skill::jig_plate_with_drill_bushings::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.bushing_id_mm    = 4.0;
    in.bushing_od_mm    = 8.0;
    in.flange_od_mm     = 11.0;
    in.flange_depth_mm  = 2.5;
    in.chamfer_mm       = 0.4;
    in.sites = {
        { 20.0, 30.0 },
        { 40.0, 30.0 },
    };

    auto out = skill::jig_plate_with_drill_bushings::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    // 4 cuts per site (with chamfer) × 2 sites = 8
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 8);
    EXPECT_NEAR(out.signature.params.at("bushing_id_mm").get<double>(),
                in.bushing_id_mm, 1e-6);
    EXPECT_NEAR(out.signature.params.at("flange_od_mm").get<double>(),
                in.flange_od_mm, 1e-6);
    EXPECT_EQ(out.signature.params.at("sites").size(), in.sites.size());
}

// ─── 4. recognize returns ≥ 1 candidate with confidence > 0.5 ───────────
TEST(SkillJigPlateWithDrillBushings, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 12.0);

    skill::jig_plate_with_drill_bushings::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.bushing_id_mm    = 5.0;
    in.bushing_od_mm    = 10.0;
    in.flange_od_mm     = 13.0;
    in.flange_depth_mm  = 3.0;
    in.chamfer_mm       = 0.5;
    in.sites = { { 25.0, 30.0 }, { 60.0, 30.0 } };

    auto out = skill::jig_plate_with_drill_bushings::apply(*stock, in);
    auto cands = skill::jig_plate_with_drill_bushings::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    // Metadata replay path → confidence == 1.0
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_EQ(cands.front().skill_id,
              std::string("jig_plate_with_drill_bushings"));
}

// ─── 5. feature-specific: site_count = sites.size() ──────────────────────
TEST(SkillJigPlateWithDrillBushings, SiteCountAndSubfeaturesPerSiteAreCorrect)
{
    auto stock = skill::createCuboidStock(120.0, 60.0, 12.0);

    skill::jig_plate_with_drill_bushings::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir         = gp_Dir(0, 0, -1);
    in.bushing_id_mm    = 5.0;
    in.bushing_od_mm    = 10.0;
    in.flange_od_mm     = 13.0;
    in.flange_depth_mm  = 3.0;
    in.chamfer_mm       = 0.5;
    in.sites = { { 20.0, 30.0 }, { 60.0, 30.0 }, { 100.0, 30.0 } };

    auto out = skill::jig_plate_with_drill_bushings::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("site_count").get<int>(), 3);
    EXPECT_EQ(out.signature.pattern.at("subfeatures_per_site").get<int>(), 4);
    // 4 × 3 = 12
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 12);
}
