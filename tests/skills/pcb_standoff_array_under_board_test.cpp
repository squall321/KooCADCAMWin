// @lat: [[process/test-strategy#compound pcb_standoff_array_under_board]]
//
// Verifies REAL multi-cut geometry: boss FUSE + pilot CUT per site, plus
// optional PCB clearance and auto-extension plate.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pcb_standoff_array_under_board.hpp"

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

// ─── T1. apply: volume delta within ±20% of expected ─────────────────────
TEST(SkillPcbStandoffArray, ApplyRealGeometryVolumeAndFaces)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 5.0);
    const int   stockFaces = stock->faceCount();
    const double stockVol  = volumeOf(stock->shape());

    skill::pcb_standoff_array_under_board::Input in;
    in.base_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.axis_dir           = gp_Dir(0, 0, 1);
    in.screw_size         = "M3";
    in.standoff_height_mm = 4.0;
    in.boss_od_mm         = 5.0;
    in.pilot_extra_depth_mm = 1.0;
    in.PCB_thickness_mm   = 0.0;
    in.plate_margin_mm    = 3.0;
    in.plate_thickness_mm = 2.0;
    in.sites = { { 20.0, 30.0 }, { 60.0, 30.0 } };

    auto out = skill::pcb_standoff_array_under_board::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected delta: per site added BOSS (π·r²·h = π·6.25·4 ≈ 78.54)
    //                 minus CUT pilot (π·(1.6/2)²·5 = π·0.64·5 ≈ 10.05)
    //                 net per site ≈ 68.5; ×2 sites ≈ 137.0
    // (Boss fuses with existing plate so we get +78.54 of new mat per site.)
    const double rBoss  = 2.5;
    const double rPilot = 0.8;  // M3 pilot = 2.5 mm dia? wait M3 -> 2.5mm, r=1.25.
    // Correct: pilotDia for M3 = 2.50 -> r = 1.25
    const double rPilotCorr = 1.25;
    const double bossH  = 4.0;
    const double pilotH = 4.0 + 1.0;  // standoff + extra
    const double perSite = M_PI * rBoss * rBoss * bossH
                         - M_PI * rPilotCorr * rPilotCorr * pilotH;
    const double expectedDelta = perSite * 2.0;
    const double netAdded = volumeOf(out.workpiece->shape()) - stockVol;
    EXPECT_NEAR(netAdded, expectedDelta, std::abs(expectedDelta) * 0.20);

    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
    (void)rPilot;
}

// ─── T2. validate rejects unknown spec + apply throws ────────────────────
TEST(SkillPcbStandoffArray, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 5.0);

    skill::pcb_standoff_array_under_board::Input in;
    in.base_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir           = gp_Dir(0, 0, 1);
    in.screw_size         = "M999";  // unknown
    in.standoff_height_mm = 4.0;
    in.boss_od_mm         = 5.0;
    in.sites = { { 25.0, 25.0 } };

    auto r = skill::pcb_standoff_array_under_board::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundSpec = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CONN-SIZE") { foundSpec = true; break; }
    EXPECT_TRUE(foundSpec);

    EXPECT_THROW(skill::pcb_standoff_array_under_board::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3. signature: is_compound + spec key + subfeature_count ────────────
TEST(SkillPcbStandoffArray, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 5.0);

    skill::pcb_standoff_array_under_board::Input in;
    in.base_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir           = gp_Dir(0, 0, 1);
    in.screw_size         = "M2.5";
    in.standoff_height_mm = 3.5;
    in.boss_od_mm         = 5.0;
    in.PCB_thickness_mm   = 1.6;     // with PCB clearance → 3 sub per site
    in.plate_thickness_mm = 2.0;
    in.sites = { { 20.0, 30.0 }, { 60.0, 30.0 } };

    auto out = skill::pcb_standoff_array_under_board::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("pcb_standoff_array_under_board"));
    EXPECT_EQ(out.signature.pattern.at("screw_size").get<std::string>(),
              std::string("M2.5"));
    // 3 sub-cuts per site (boss + pilot + PCB clearance) × 2 = 6
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 6);
    EXPECT_EQ(out.signature.pattern.at("site_count").get<int>(), 2);
}

// ─── T4. recognize returns ≥1 candidate with the right spec key ─────────
TEST(SkillPcbStandoffArray, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(80.0, 60.0, 5.0);

    skill::pcb_standoff_array_under_board::Input in;
    in.base_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir           = gp_Dir(0, 0, 1);
    in.screw_size         = "M3";
    in.standoff_height_mm = 4.0;
    in.boss_od_mm         = 5.0;
    in.plate_thickness_mm = 2.0;
    in.sites = { { 25.0, 30.0 }, { 60.0, 30.0 } };

    auto out = skill::pcb_standoff_array_under_board::apply(*stock, in);
    auto cands = skill::pcb_standoff_array_under_board::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);    // metadata replay path
    EXPECT_EQ(cands.front().skill_id,
              std::string("pcb_standoff_array_under_board"));
    EXPECT_EQ(cands.front().recovered_params.at("screw_size").get<std::string>(),
              std::string("M3"));
}

// ─── T5. CONTEXT: auto-extend plate when sites fall outside footprint ───
TEST(SkillPcbStandoffArray, AutoExtendsBasePlateWhenSitesOutsideFootprint)
{
    // Small 30 × 30 plate; place sites OUTSIDE the footprint.
    auto stock = skill::createCuboidStock(30.0, 30.0, 3.0);

    skill::pcb_standoff_array_under_board::Input in;
    in.base_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir           = gp_Dir(0, 0, 1);
    in.screw_size         = "M3";
    in.standoff_height_mm = 4.0;
    in.boss_od_mm         = 5.0;
    in.plate_margin_mm    = 3.0;
    in.plate_thickness_mm = 2.0;
    // Place a site WAY outside the original 30 × 30 footprint
    in.sites = { { 15.0, 15.0 }, { 50.0, 15.0 } };

    auto out = skill::pcb_standoff_array_under_board::apply(*stock, in);
    EXPECT_TRUE(out.signature.params.at("auto_extended_plate").get<bool>());
    EXPECT_TRUE(out.signature.pattern.at("auto_extended_plate").get<bool>());

    // Verify the resulting workpiece bbox actually grew along +X.
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_GT(xMax, 30.0);
}
