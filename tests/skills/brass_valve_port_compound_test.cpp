// @lat: [[process/test-strategy#brass_valve_port_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/brass_valve_port_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::brass_valve_port_compound::Input goodInput()
{
    skill::brass_valve_port_compound::Input in;
    in.face_id           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm       = 50.0;
    in.center_y_mm       = 50.0;
    in.valve_axis_dir    = gp_Dir(0, 0, 1);
    in.valve_bore_dia_mm = 13.0;
    in.slide_port_count  = 3;
    in.slide_port_dia_mm = 11.0;
    in.slide_port_pitch_deg = 120.0;
    in.slide_port_radial_distance_mm = 20.0;
    in.slide_port_depth_mm = 15.0;
    return in;
}
}  // namespace

// Test 1: apply removes material (bore + 3 slide ports).
TEST(SkillBrassValvePortCompound, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 40.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::brass_valve_port_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 0.0);
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillBrassValvePortCompound, ValidateRejectsPortCountOutOfRange)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 40.0);
    auto in = goodInput();
    in.slide_port_count = 1;   // < 2 minimum

    auto r = skill::brass_valve_port_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PORT-COUNT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::brass_valve_port_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillBrassValvePortCompound, SignatureRecordsCompoundAndPortCount)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::brass_valve_port_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("brass_valve_port_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("audio_feature_type", std::string()),
              std::string("brass_valve_port"));
    EXPECT_EQ(out.signature.pattern.value("slide_port_count", 0),
              in.slide_port_count);
    // subfeature_count = 1 (bore) + N (ports) = 4
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 4);
}

TEST(SkillBrassValvePortCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 40.0);
    auto in    = goodInput();
    auto out   = skill::brass_valve_port_compound::apply(*stock, in);
    auto cands = skill::brass_valve_port_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.value("slide_port_count", 0),
              in.slide_port_count);
}
