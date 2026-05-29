// @lat: [[process/test-strategy#compound feature]]
//
// usb_c_port_cutout — 4-cut compound: main pocket + EMI lip groove + 2
// retention notches.
//
// Test cases:
//   1. ApplyProducesValidShapeWithMultipleSubFeatures
//   2. ValidateRejectsBadInput — panel too thick (USB-IF SP-30 max 3 mm)
//   3. SignatureRecordsCompoundKind
//   4. RecognizeMetadataReplay
//   5. PocketDimensionsMatchUsbIfSp30 — 8.94 × 2.56 mm

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/usb_c_port_cutout.hpp"

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

// ─── 1. Apply produces valid shape ────────────────────────────────────────
TEST(SkillUsbCPortCutout, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(40.0, 25.0, 2.0);  // 2 mm panel

    skill::usb_c_port_cutout::Input in;
    in.face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm   = 20.0;
    in.position_y_mm   = 12.5;
    in.axis_dir        = gp_Dir(0, 0, -1);
    in.orientation_deg = 0.0;

    const int facesBefore = stock->faceCount();
    auto out = skill::usb_c_port_cutout::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Main pocket 8.94 × 2.56 × 2.0 ≈ 45.8 mm³
    //  + EMI groove (perimeter ≈ 22.9 × 0.5 × 0.4 ≈ 4.6 mm³)
    //  + 2 notches (0.8 × 0.5 × 1.5 each ≈ 0.6 mm³ each)
    const double approx  = 8.94 * 2.56 * 2.0 + 4.6 + 1.2;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    // Wide tolerance — EMI annular geometry & overlap with pocket.
    EXPECT_NEAR(removed, approx, approx * 0.4);

    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("usb_c_port_cutout"));
}

// ─── 2. Validate rejects too-thick panel ──────────────────────────────────
TEST(SkillUsbCPortCutout, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 25.0, 5.0);  // > 3 mm SP-30 max

    skill::usb_c_port_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 12.5;

    auto r = skill::usb_c_port_cutout::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::usb_c_port_cutout::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillUsbCPortCutout, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(40.0, 25.0, 2.0);
    skill::usb_c_port_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 12.5;
    auto out = skill::usb_c_port_cutout::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_TRUE(pat.at("is_compound").get<bool>());
    EXPECT_GE(pat.at("subfeature_count").get<int>(), 2);
    EXPECT_EQ(pat.at("kind").get<std::string>(),
              std::string("usb_c_port_cutout"));
    EXPECT_EQ(pat.at("subfeatures").size(), 4u);
}

// ─── 4. Recognize: metadata replay → confidence 1.0 ───────────────────────
TEST(SkillUsbCPortCutout, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 25.0, 2.0);
    skill::usb_c_port_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 12.0;
    in.position_y_mm = 12.5;
    auto out = skill::usb_c_port_cutout::apply(*stock, in);

    auto cands = skill::usb_c_port_cutout::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["position_x_mm"].get<double>(),
                12.0, 1e-6);
}

// ─── 5. Pocket dimensions match USB-IF SP-30 (8.94 × 2.56) ────────────────
TEST(SkillUsbCPortCutout, PocketDimensionsMatchUsbIfSp30)
{
    auto stock = skill::createCuboidStock(40.0, 25.0, 2.0);
    skill::usb_c_port_cutout::Input in;
    in.face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 20.0;
    in.position_y_mm = 12.5;
    auto out = skill::usb_c_port_cutout::apply(*stock, in);

    const auto& pat = out.signature.pattern;
    EXPECT_NEAR(pat.at("pocket_long_mm").get<double>(),    8.94, 0.05);
    EXPECT_NEAR(pat.at("pocket_short_mm").get<double>(),   2.56, 0.05);
    // Corner R = half of short side (rounded-end pill)
    EXPECT_NEAR(pat.at("pocket_corner_r_mm").get<double>(), 1.28, 0.05);
    EXPECT_NEAR(pat.at("emi_groove_width_mm").get<double>(), 0.5, 0.05);
}
