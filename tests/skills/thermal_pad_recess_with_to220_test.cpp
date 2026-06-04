// @lat: [[process/test-strategy#thermal_pad_recess_with_to220]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/thermal_pad_recess_with_to220.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ─── 1. apply REMOVES recess + screw hole volume ────────────────────────
TEST(SkillThermalPadTO220, ApplyRemovesRecessAndScrew)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    const double v0 = volumeOf(stock->shape());

    skill::thermal_pad_recess_with_to220::Input in;
    in.face_id               = 0;
    in.center_x_mm           = 20.0;
    in.center_y_mm           = 20.0;
    in.device_package        = "TO-220";
    in.pad_thickness_mm      = 0.5;
    in.screw_thread_size_key = "M3";
    auto out = skill::thermal_pad_recess_with_to220::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0);  // material was removed
    // Recess is ~10.4 × 5.0 × 0.6 = 31.2 mm³; screw drill ~ π·(1.7)²·8 ≈ 72.6 mm³.
    const double removed = v0 - v1;
    EXPECT_GT(removed, 80.0);
    EXPECT_LT(removed, 200.0);

    EXPECT_EQ(out.signature.skill_id,
              std::string("thermal_pad_recess_with_to220"));
}

// ─── 2. validate rejects bad package + bad screw key ────────────────────
TEST(SkillThermalPadTO220, ValidateRejectsBadInputs)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);

    {   // (a) unknown package
        skill::thermal_pad_recess_with_to220::Input in;
        in.device_package        = "TO-99";
        in.pad_thickness_mm      = 0.5;
        in.screw_thread_size_key = "M3";
        auto r = skill::thermal_pad_recess_with_to220::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-PKG") found = true;
        EXPECT_TRUE(found);
        EXPECT_THROW(skill::thermal_pad_recess_with_to220::apply(*stock, in),
                     skill::SkillError);
    }
    {   // (b) unknown screw key
        skill::thermal_pad_recess_with_to220::Input in;
        in.device_package        = "TO-247";
        in.pad_thickness_mm      = 0.5;
        in.screw_thread_size_key = "M99";
        auto r = skill::thermal_pad_recess_with_to220::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-THREAD-KEY") found = true;
        EXPECT_TRUE(found);
    }
}

// ─── 3. signature pattern carries package + screw + dims ────────────────
TEST(SkillThermalPadTO220, SignatureCarriesPackageAndScrew)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    skill::thermal_pad_recess_with_to220::Input in;
    in.center_x_mm = 20.0; in.center_y_mm = 20.0;
    in.device_package = "TO-247"; in.pad_thickness_mm = 0.8;
    in.screw_thread_size_key = "M4";
    auto out = skill::thermal_pad_recess_with_to220::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("electronics_feature_type").get<std::string>(),
              std::string("power_device_thermal_mount"));
    EXPECT_EQ(out.signature.pattern.at("device_package").get<std::string>(),
              std::string("TO-247"));
    EXPECT_EQ(out.signature.pattern.at("screw_thread_size_key").get<std::string>(),
              std::string("M4"));
    EXPECT_NEAR(out.signature.pattern.at("footprint_w_mm").get<double>(), 16.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern.at("footprint_d_mm").get<double>(),  5.5, 1e-6);
}

// ─── 4. recognize from metadata ──────────────────────────────────────────
TEST(SkillThermalPadTO220, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    skill::thermal_pad_recess_with_to220::Input in;
    in.center_x_mm = 20.0; in.center_y_mm = 20.0;
    in.device_package = "D2PAK"; in.pad_thickness_mm = 0.5;
    in.screw_thread_size_key = "M3";
    auto out = skill::thermal_pad_recess_with_to220::apply(*stock, in);

    auto cands = skill::thermal_pad_recess_with_to220::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("device_package").get<std::string>(),
              std::string("D2PAK"));
}

// ─── 5. footprint table lookup correctness ──────────────────────────────
TEST(SkillThermalPadTO220, FootprintTableCorrectness)
{
    auto to220 = skill::thermal_pad_recess_with_to220::footprintFor("TO-220");
    EXPECT_NEAR(to220.width_mm, 10.0, 1e-9);
    EXPECT_NEAR(to220.depth_mm,  4.6, 1e-9);
    EXPECT_STREQ(to220.recommended_screw, "M3");

    auto to247 = skill::thermal_pad_recess_with_to220::footprintFor("TO-247");
    EXPECT_NEAR(to247.width_mm, 16.0, 1e-9);
    EXPECT_STREQ(to247.recommended_screw, "M4");

    auto bad = skill::thermal_pad_recess_with_to220::footprintFor("XYZ");
    EXPECT_NEAR(bad.width_mm, 0.0, 1e-9);
}
