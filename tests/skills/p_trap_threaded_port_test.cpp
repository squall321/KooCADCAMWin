// @lat: [[process/test-strategy#p_trap_threaded_port]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/p_trap_threaded_port.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::p_trap_threaded_port::Input goodInput()
{
    skill::p_trap_threaded_port::Input in;
    in.face_id                = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm            = 50.0;
    in.center_y_mm            = 40.0;
    in.trap_radius_mm         = 30.0;    // > 1.5 × NPT-1/2 drill (17.85mm)
    in.inlet_thread_size_key  = "1/2";   // NPT 1/2
    in.outlet_thread_size_key = "1/2";
    return in;
}
}  // namespace

TEST(SkillPTrapThreadedPort, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 60.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::p_trap_threaded_port::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (2 NPT ports + U-channel)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillPTrapThreadedPort, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 60.0);
    auto in = goodInput();
    in.inlet_thread_size_key = "BOGUS";

    auto r = skill::p_trap_threaded_port::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NPT-CODE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::p_trap_threaded_port::apply(*stock, in), skill::SkillError);
}

TEST(SkillPTrapThreadedPort, SignatureCompoundFiveSubfeatures)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::p_trap_threaded_port::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("p_trap_threaded_port"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("hvac_feature_type", std::string()),
              std::string("p_trap"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 5);
}

TEST(SkillPTrapThreadedPort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 60.0);
    auto in    = goodInput();
    auto out   = skill::p_trap_threaded_port::apply(*stock, in);
    auto cands = skill::p_trap_threaded_port::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
