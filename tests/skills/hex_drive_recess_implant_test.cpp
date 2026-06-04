// @lat: [[process/test-strategy#medical hex driver recess]]
//
// hex_drive_recess_implant — implant hex driver socket (6-sided polygon pocket).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hex_drive_recess_implant.hpp"

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

// ─── 1. apply produces hex pocket with correct removed volume ────────────
TEST(SkillHexDriveRecess, ApplyProducesHexPocket)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 6.0);
    const double stockVol = volumeOf(stock->shape());
    const int    stockFC  = stock->faceCount();

    skill::hex_drive_recess_implant::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm = 10.0;
    in.position_y_mm = 10.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.hex_AF_mm     = 2.5;       // standard implant driver AF
    in.depth_mm      = 3.0;

    auto out = skill::hex_drive_recess_implant::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Regular hexagon area = (√3 / 2) · AF² = 0.866 · 6.25 ≈ 5.41
    // Removed ≈ 5.41 × 3 ≈ 16.24 mm³  (plus 0.05 overshoot)
    const double removed = stockVol - volumeOf(out.workpiece->shape());
    const double hexArea = std::sqrt(3.0) / 2.0 * 2.5 * 2.5;
    const double expected = hexArea * 3.0;
    EXPECT_NEAR(removed, expected, expected * 0.10);

    EXPECT_GT(out.workpiece->faceCount(), stockFC);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("hex_drive_recess_implant"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("medical_feature_type").get<std::string>(),
              std::string("implant_hex_driver_socket"));
    EXPECT_EQ(out.signature.pattern.at("side_count").get<int>(), 6);
    EXPECT_NEAR(out.signature.pattern.at("hex_AF_mm").get<double>(), 2.5, 1e-6);
}

// ─── 2. validate rejects non-standard AF ─────────────────────────────────
TEST(SkillHexDriveRecess, ValidateRejectsNonStandardAF)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 6.0);

    skill::hex_drive_recess_implant::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0; in.position_y_mm = 10.0;
    in.hex_AF_mm = 3.5;        // NOT in {1.5, 2.0, 2.5, 3.0, 4.0, 5.0}
    in.depth_mm  = 3.0;

    auto r = skill::hex_drive_recess_implant::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HEX-AF") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::hex_drive_recess_implant::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects zero depth ──────────────────────────────────────
TEST(SkillHexDriveRecess, ValidateRejectsZeroDepth)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 6.0);

    skill::hex_drive_recess_implant::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0; in.position_y_mm = 10.0;
    in.hex_AF_mm = 2.0;
    in.depth_mm  = 0.0;

    auto r = skill::hex_drive_recess_implant::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HEX-DEPTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. recognize metadata replay ────────────────────────────────────────
TEST(SkillHexDriveRecess, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 6.0);

    skill::hex_drive_recess_implant::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 10.0; in.position_y_mm = 10.0;
    in.hex_AF_mm = 4.0;
    in.depth_mm  = 4.0;

    auto out = skill::hex_drive_recess_implant::apply(*stock, in);
    auto cands = skill::hex_drive_recess_implant::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_NEAR(cands[0].recovered_params.at("hex_AF_mm").get<double>(), 4.0, 1e-6);
}

// ─── 5. all six standard AF values accepted ──────────────────────────────
TEST(SkillHexDriveRecess, AllStandardAFAccepted)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 8.0);

    for (double af : { 1.5, 2.0, 2.5, 3.0, 4.0, 5.0 }) {
        skill::hex_drive_recess_implant::Input in;
        in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = 10.0; in.position_y_mm = 10.0;
        in.hex_AF_mm = af;
        in.depth_mm  = 2.0;
        auto r = skill::hex_drive_recess_implant::validate(*stock, in);
        EXPECT_TRUE(r.passed) << "AF " << af << " should pass";
    }
}
