// @lat: [[process/test-strategy#compound fastener-seat]]
//
// captive_nut_pocket — COMPOUND macro: hex pocket + access slot + spring
// detent relief, all at the same depth.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/captive_nut_pocket.hpp"

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

// ─── 1. apply produces valid shape with N sub-features ───────────────────
TEST(SkillCaptiveNutPocket, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);
    const int stockFaceCount = stock->faceCount();
    const double stockVol = volumeOf(stock->shape());

    skill::captive_nut_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 30.0;
    in.position_y_mm = 25.0;
    in.nut_size      = "M4";
    in.slip_fit_mm   = 0.2;
    in.entry_edge    = "+x";

    auto out = skill::captive_nut_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Some material should have been removed (rough lower bound check).
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, 5.0);     // > 5 mm³ at minimum
    EXPECT_LT(removed, 5000.0);  // upper sanity bound

    EXPECT_GT(out.workpiece->faceCount(), stockFaceCount);

    EXPECT_EQ(out.signature.skill_id, std::string("captive_nut_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillCaptiveNutPocket, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    // (a) Unknown nut size
    {
        skill::captive_nut_pocket::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0; in.position_y_mm = 25.0;
        in.nut_size = "M99";
        auto r = skill::captive_nut_pocket::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        EXPECT_THROW(skill::captive_nut_pocket::apply(*stock, in), skill::SkillError);
    }
    // (b) Bad entry_edge
    {
        skill::captive_nut_pocket::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0; in.position_y_mm = 25.0;
        in.nut_size = "M3";
        in.entry_edge = "diagonal";
        auto r = skill::captive_nut_pocket::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-NUT-EDGE") { found = true; break; }
        EXPECT_TRUE(found);
    }
    // (c) Stock too thin
    {
        auto thin = skill::createCuboidStock(50.0, 50.0, 1.0);
        skill::captive_nut_pocket::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 25.0; in.position_y_mm = 25.0;
        in.nut_size = "M6";       // height 5 mm → pocket 5.2 > 1
        auto r = skill::captive_nut_pocket::validate(*thin, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-NUT-DEPTH") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 3. Signature records compound kind ──────────────────────────────────
TEST(SkillCaptiveNutPocket, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::captive_nut_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 25.0;
    in.nut_size      = "M3";
    in.entry_edge    = "+x";

    auto out = skill::captive_nut_pocket::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("captive_nut_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}

// ─── 4. recognize replays metadata at confidence 1.0 ─────────────────────
TEST(SkillCaptiveNutPocket, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);

    skill::captive_nut_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 25.0;
    in.nut_size      = "M4";
    in.entry_edge    = "+x";

    auto out = skill::captive_nut_pocket::apply(*stock, in);
    auto cands = skill::captive_nut_pocket::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("nut_size").get<std::string>(),
              std::string("M4"));
    EXPECT_EQ(cands[0].recovered_params.at("entry_edge").get<std::string>(),
              std::string("+x"));
}

// ─── 5. Specific: hex flat-to-flat matches M-size table ──────────────────
TEST(SkillCaptiveNutPocket, HexFlatToFlatMatchesIsoTable)
{
    // ISO 4032 hex-nut AF (across-flats) values.
    EXPECT_NEAR(skill::captive_nut_pocket::nutAcrossFlatsFor("M3"), 5.5,  1e-3);
    EXPECT_NEAR(skill::captive_nut_pocket::nutAcrossFlatsFor("M4"), 7.0,  1e-3);
    EXPECT_NEAR(skill::captive_nut_pocket::nutAcrossFlatsFor("M5"), 8.0,  1e-3);
    EXPECT_NEAR(skill::captive_nut_pocket::nutAcrossFlatsFor("M6"), 10.0, 1e-3);
    EXPECT_EQ  (skill::captive_nut_pocket::nutAcrossFlatsFor("M99"), 0.0);

    EXPECT_NEAR(skill::captive_nut_pocket::nutHeightFor("M4"), 3.2, 1e-3);
    EXPECT_NEAR(skill::captive_nut_pocket::nutHeightFor("M6"), 5.0, 1e-3);

    // Apply produces pocket_af = AF + slip_fit.
    auto stock = skill::createCuboidStock(50.0, 50.0, 15.0);
    skill::captive_nut_pocket::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0; in.position_y_mm = 25.0;
    in.nut_size      = "M4";    // AF 7.0
    in.slip_fit_mm   = 0.2;
    in.entry_edge    = "+x";
    auto out = skill::captive_nut_pocket::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern.at("nut_af_mm").get<double>(),    7.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("pocket_af_mm").get<double>(), 7.2, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("pocket_depth_mm").get<double>(),
                3.2 + 0.2, 1e-6);  // nut_height + 0.2 mm clearance
}
