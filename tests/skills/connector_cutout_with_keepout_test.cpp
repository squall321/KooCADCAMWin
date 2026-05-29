// @lat: [[process/test-strategy#compound connector_cutout_with_keepout]]
//
// Verifies REAL multi-cut geometry: rounded-rect through pocket +
// chamfer + retention notches, per the spec table.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/connector_cutout_with_keepout.hpp"

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

// ─── T1. apply: volume delta matches USB-A pocket (±20%) ────────────────
TEST(SkillConnectorCutout, ApplyRealGeometryVolumeAndFaces)
{
    // 60 × 40 × 2 panel
    auto stock = skill::createCuboidStock(60.0, 40.0, 2.0);
    const int    stockFaces = stock->faceCount();
    const double stockVol   = volumeOf(stock->shape());

    skill::connector_cutout_with_keepout::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.connector_type  = "USB-A";
    in.position_x_mm   = 30.0;
    in.position_y_mm   = 20.0;

    auto out = skill::connector_cutout_with_keepout::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // USB-A: body 12.0 × 4.5, thickness 2.0 → pocket volume ≈ 108
    // Plus 2 retention notches 1.5 × 0.4 × 2 = 1.2 each ≈ 2.4 → ~110
    const double approx = 12.0 * 4.5 * 2.0 + 2.0 * (1.5 * 0.4 * 2.0);
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, approx, approx * 0.20);

    EXPECT_GT(out.workpiece->faceCount(), stockFaces);
}

// ─── T2. validate rejects unknown connector_type + apply throws ─────────
TEST(SkillConnectorCutout, ValidateRejectsUnknownSpec)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 2.0);

    skill::connector_cutout_with_keepout::Input in;
    in.face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.connector_type = "USB-Z";   // unknown
    in.position_x_mm  = 30.0;
    in.position_y_mm  = 20.0;

    auto r = skill::connector_cutout_with_keepout::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool foundSpec = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CONN-SIZE") { foundSpec = true; break; }
    EXPECT_TRUE(foundSpec);

    EXPECT_THROW(skill::connector_cutout_with_keepout::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3. signature carries spec key + is_compound + EMI keepout meta ────
TEST(SkillConnectorCutout, SignatureCarriesSpecAndCompoundFlag)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 2.0);

    skill::connector_cutout_with_keepout::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.connector_type  = "HDMI";
    in.position_x_mm   = 30.0;
    in.position_y_mm   = 20.0;

    auto out = skill::connector_cutout_with_keepout::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("connector_cutout_with_keepout"));
    EXPECT_EQ(out.signature.pattern.at("connector_type").get<std::string>(),
              std::string("HDMI"));
    EXPECT_GE(out.signature.pattern.at("subfeature_count").get<int>(), 2);
    // EMI keepout metadata must be present.
    ASSERT_TRUE(out.signature.pattern.contains("emi_keepout"));
    EXPECT_EQ(
        out.signature.pattern.at("emi_keepout").at("emi_standard").get<std::string>(),
        std::string("IPC-2221"));
}

// ─── T4. recognize returns metadata-replay candidate ────────────────────
TEST(SkillConnectorCutout, RecognizeReturnsCandidate)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 2.0);

    skill::connector_cutout_with_keepout::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.connector_type  = "RJ45";
    in.position_x_mm   = 30.0;
    in.position_y_mm   = 20.0;

    auto out = skill::connector_cutout_with_keepout::apply(*stock, in);
    auto cands = skill::connector_cutout_with_keepout::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands.front().confidence, 1.0);
    EXPECT_EQ(cands.front().skill_id,
              std::string("connector_cutout_with_keepout"));
    EXPECT_EQ(
        cands.front().recovered_params.at("connector_type").get<std::string>(),
        std::string("RJ45"));
}

// ─── T5. SPEC TABLE: D-sub-9 has 0 retention notches per spec ───────────
TEST(SkillConnectorCutout, SpecTableDSub9HasZeroRetentionNotches)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 2.0);

    skill::connector_cutout_with_keepout::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.connector_type  = "D-sub-9";
    in.position_x_mm   = 30.0;
    in.position_y_mm   = 20.0;

    auto out = skill::connector_cutout_with_keepout::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern.at("retention_notch_count").get<int>(), 0);
    // pocket + chamfer = 2 subfeatures (no notches for D-sub-9)
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 2);
}
