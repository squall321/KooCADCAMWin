// @lat: [[process/test-strategy#hydraulic_quick_coupler_iso7241]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hydraulic_quick_coupler_iso7241.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::hydraulic_quick_coupler_iso7241::Input goodInput()
{
    skill::hydraulic_quick_coupler_iso7241::Input in;
    in.face_id         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy       = gp_Pnt(30.0, 30.0, 0.0);
    in.dash_size       = "-6";
    in.thread_size_key = "G3/8";
    return in;
}
}  // namespace

TEST(SkillHydraulicQuickCouplerIso7241, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::hydraulic_quick_coupler_iso7241::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // material removed (port + groove + spotface)
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillHydraulicQuickCouplerIso7241, ValidateRejectsInvalidDash)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in = goodInput();
    in.dash_size = "-99";

    auto r = skill::hydraulic_quick_coupler_iso7241::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-DASH") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::hydraulic_quick_coupler_iso7241::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillHydraulicQuickCouplerIso7241, ValidateRejectsUnknownThread)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in = goodInput();
    in.thread_size_key = "G99";

    auto r = skill::hydraulic_quick_coupler_iso7241::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-THREAD") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillHydraulicQuickCouplerIso7241, SignatureCompoundIso7241)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::hydraulic_quick_coupler_iso7241::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("hydraulic_quick_coupler_iso7241"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("ag_feature_type", std::string()),
              std::string("iso7241_flat_face_quick_coupler"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
    EXPECT_EQ(out.signature.pattern.value("standard", std::string()),
              std::string("ISO 7241-A"));
}

TEST(SkillHydraulicQuickCouplerIso7241, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 30.0);
    auto in    = goodInput();
    auto out   = skill::hydraulic_quick_coupler_iso7241::apply(*stock, in);
    auto cands = skill::hydraulic_quick_coupler_iso7241::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
