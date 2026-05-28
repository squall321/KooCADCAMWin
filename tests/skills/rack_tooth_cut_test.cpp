// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rack_tooth_cut.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}

TEST(SkillRackToothCut, ApplyChangesGeometry)
{
    auto stock = skill::createCuboidStock(100.0, 20.0, 10.0);

    skill::rack_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 50.0;
    in.position_y_mm    = 10.0;
    in.module_mm        = 2.0;
    in.pressure_angle_deg = 20.0;
    in.depth_mm         = 2.0;
    in.width_mm         = M_PI;       // half pitch ≈ 3.14
    in.length_mm        = 20.0;
    in.length_dir       = gp_Dir(0, 1, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::rack_tooth_cut::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_LT(v1, v0);
    const double approxRemoved = in.length_mm * in.width_mm * in.depth_mm;
    EXPECT_NEAR(v0 - v1, approxRemoved, approxRemoved * 0.05);
}

TEST(SkillRackToothCut, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 20.0, 10.0);

    skill::rack_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.module_mm        = 60.0;   // out of [0.2, 50.0]
    in.pressure_angle_deg = 20.0;
    in.depth_mm         = 2.0;
    in.width_mm         = 3.0;
    in.length_mm        = 20.0;

    auto r = skill::rack_tooth_cut::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GEAR-MOD") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::rack_tooth_cut::apply(*stock, in), skill::SkillError);
}

TEST(SkillRackToothCut, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(100.0, 20.0, 10.0);

    skill::rack_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 50.0;
    in.position_y_mm    = 10.0;
    in.module_mm        = 2.0;
    in.pressure_angle_deg = 20.0;
    in.depth_mm         = 2.0;
    in.width_mm         = 3.14;
    in.length_mm        = 20.0;
    in.length_dir       = gp_Dir(0, 1, 0);

    auto out = skill::rack_tooth_cut::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("rack_tooth_cut"));
    EXPECT_EQ(out.signature.pattern["kind"], "rack_tooth_cut");
    EXPECT_NEAR(out.signature.params["module_mm"].get<double>(), 2.0, 1e-9);
    EXPECT_NEAR(out.signature.params["pressure_angle_deg"].get<double>(), 20.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["pitch_mm"].get<double>(),
                M_PI * 2.0, 1e-3);
}

TEST(SkillRackToothCut, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 20.0, 10.0);

    skill::rack_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 50.0;
    in.position_y_mm    = 10.0;
    in.module_mm        = 1.5;
    in.pressure_angle_deg = 20.0;
    in.depth_mm         = 1.5;
    in.width_mm         = 2.0;
    in.length_mm        = 18.0;
    in.length_dir       = gp_Dir(0, 1, 0);

    auto out = skill::rack_tooth_cut::apply(*stock, in);
    auto cands = skill::rack_tooth_cut::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("rack_tooth_cut"));
    EXPECT_NEAR(cands[0].recovered_params["module_mm"].get<double>(),
                1.5, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["length_mm"].get<double>(),
                18.0, 1e-9);
}

// ─── 5. Specific: stock_removed equals length × width × depth ─────────────
TEST(SkillRackToothCut, ToolingStockRemovedMatchesGeometry)
{
    auto stock = skill::createCuboidStock(100.0, 20.0, 10.0);

    skill::rack_tooth_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm    = 50.0;
    in.position_y_mm    = 10.0;
    in.module_mm        = 2.0;
    in.pressure_angle_deg = 20.0;
    in.depth_mm         = 2.0;
    in.width_mm         = 3.0;
    in.length_mm        = 15.0;
    in.length_dir       = gp_Dir(1, 0, 0);

    auto out = skill::rack_tooth_cut::apply(*stock, in);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3,
                15.0 * 3.0 * 2.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("end_mill"));
}
