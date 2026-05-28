// @lat: [[process/test-strategy#skill round-trip]]
//
// chuck_grip_zone skill — 5-case sweep.  Metadata-only.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/chuck_grip_zone.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply: geometry unchanged + signature added ───────────────────────
TEST(SkillChuckGripZone, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::chuck_grip_zone::Input in;
    in.cylinder_axis = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
    in.grip_length   = 15.0;

    auto out = skill::chuck_grip_zone::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("chuck_grip_zone"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate: non-positive grip_length rejected ───────────────────────
TEST(SkillChuckGripZone, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);

    skill::chuck_grip_zone::Input in;
    in.cylinder_axis = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
    in.grip_length   = 0.0;            // invalid

    auto r = skill::chuck_grip_zone::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::chuck_grip_zone::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records cylinder_axis + grip_length ─────────────────────
TEST(SkillChuckGripZone, SignatureRecordsKindAndKeys)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);

    skill::chuck_grip_zone::Input in;
    in.cylinder_axis = gp_Ax1(gp_Pnt(1.0, 2.0, 3.0), gp::DZ());
    in.grip_length   = 20.0;

    auto out = skill::chuck_grip_zone::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("chuck_grip_zone"));
    ASSERT_TRUE(out.signature.pattern.contains("cylinder_axis"));
    ASSERT_TRUE(out.signature.pattern["cylinder_axis"].contains("origin"));
    ASSERT_TRUE(out.signature.pattern["cylinder_axis"].contains("direction"));
    EXPECT_NEAR(out.signature.pattern["grip_length"].get<double>(), 20.0, 1e-6);
    // Origin echoes back unchanged.
    const auto& origin = out.signature.pattern["cylinder_axis"]["origin"];
    EXPECT_NEAR(origin[0].get<double>(), 1.0, 1e-9);
    EXPECT_NEAR(origin[1].get<double>(), 2.0, 1e-9);
    EXPECT_NEAR(origin[2].get<double>(), 3.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillChuckGripZone, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);

    skill::chuck_grip_zone::Input in;
    in.cylinder_axis = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
    in.grip_length   = 12.0;
    auto out = skill::chuck_grip_zone::apply(*stock, in);

    auto cands = skill::chuck_grip_zone::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["grip_length"].get<double>(),
                12.0, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::chuck_grip_zone::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific assertion: short grip_length emits warning, still passes ─
TEST(SkillChuckGripZone, ShortGripIsWarningNotError)
{
    auto stock = skill::createCylindricalStock(30.0, 100.0);

    skill::chuck_grip_zone::Input in;
    in.cylinder_axis = gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp::DZ());
    in.grip_length   = 1.0;            // < 3 mm → warning

    auto r = skill::chuck_grip_zone::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "CG-LENGTH-SHORT"));
    EXPECT_NO_THROW(skill::chuck_grip_zone::apply(*stock, in));
}
