// @lat: [[process/test-strategy#internal_cable_port_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/internal_cable_port_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

skill::internal_cable_port_compound::Input goodInput()
{
    skill::internal_cable_port_compound::Input in;
    in.face_id                 = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_xy               = gp_Pnt(25.0, 25.0, 0.0);
    in.port_dia_mm             = 6.0;
    in.grommet_groove_dia_mm   = 10.0;
    in.grommet_groove_depth_mm = 1.2;
    in.entry_angle_deg         = 30.0;
    return in;
}
}  // namespace

TEST(SkillInternalCablePortCompound, ApplyRemovesMaterial)
{
    // Frame wall block: 50 x 50 x 14 mm; angled port at (25, 25).
    auto stock = skill::createCuboidStock(50.0, 50.0, 14.0);
    const double v0 = volumeOf(stock->shape());
    auto in  = goodInput();
    auto out = skill::internal_cable_port_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);   // angled bore + grommet groove
    EXPECT_GT(out.workpiece->faceCount(), stock->faceCount());
}

TEST(SkillInternalCablePortCompound, ValidateRejectsSteepAngle)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 14.0);
    auto in = goodInput();
    in.entry_angle_deg = 80.0;   // out of [0, 60]

    auto r = skill::internal_cable_port_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::internal_cable_port_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillInternalCablePortCompound, ValidateRejectsSmallGroove)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 14.0);
    auto in = goodInput();
    in.grommet_groove_dia_mm = 5.0;   // <= port_dia_mm

    auto r = skill::internal_cable_port_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GROOVE") { found = true; break; }
    EXPECT_TRUE(found);
}

TEST(SkillInternalCablePortCompound, SignatureCompoundCablePort)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 14.0);
    auto in    = goodInput();
    auto out   = skill::internal_cable_port_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("internal_cable_port_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("bicycle_feature_type", std::string()),
              std::string("internal_cable_routing_port"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 2);
}

TEST(SkillInternalCablePortCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 14.0);
    auto in    = goodInput();
    auto out   = skill::internal_cable_port_compound::apply(*stock, in);
    auto cands = skill::internal_cable_port_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
