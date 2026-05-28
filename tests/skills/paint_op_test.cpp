// @lat: [[process/test-strategy#skill round-trip]]
//
// paint_op skill — 6-case sweep covering identity-geometry synthesis,
// thickness DFM clamps, powder→powder_coat warning, metadata recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/paint_op.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillPaintOp, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::paint_op::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.paint_type    = "wet";
    in.color         = "red";
    in.thickness_um  = 50.0;
    in.cure_temp_c   = 80.0;
    in.cure_time_min = 20.0;

    auto out = skill::paint_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);

    EXPECT_EQ(out.signature.skill_id, std::string("paint_op"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("spray_gun"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: thickness [10, 500] μm enforced ─────────────────────────────
TEST(SkillPaintOp, ValidateRejectsOutOfRangeThickness)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::paint_op::Input tooThin;
    tooThin.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooThin.thickness_um = 5.0;
    EXPECT_FALSE(skill::paint_op::validate(*stock, tooThin).passed);
    EXPECT_THROW(skill::paint_op::apply(*stock, tooThin), skill::SkillError);

    skill::paint_op::Input tooThick = tooThin;
    tooThick.thickness_um = 800.0;
    EXPECT_FALSE(skill::paint_op::validate(*stock, tooThick).passed);
    EXPECT_THROW(skill::paint_op::apply(*stock, tooThick), skill::SkillError);
}

// ─── 3. paint_type='powder' emits warning (route to powder_coat) ─────────
TEST(SkillPaintOp, PowderTypeWarning)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::paint_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.paint_type   = "powder";       // expected: route to powder_coat skill
    in.color        = "black";
    in.thickness_um = 75.0;

    auto r = skill::paint_op::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // warning, not error
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PAINT-TYPE" && f.severity == "warning") {
            found = true; break;
        }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::paint_op::apply(*stock, in));
}

// ─── 4. Signature carries color + cure schedule metadata ─────────────────
TEST(SkillPaintOp, SignatureRecordsMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::paint_op::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.paint_type    = "wet";
    in.color         = "matte_white";
    in.thickness_um  = 100.0;
    in.cure_temp_c   = 100.0;
    in.cure_time_min = 30.0;

    auto out = skill::paint_op::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("paint_op"));
    EXPECT_EQ(out.signature.pattern["color"].get<std::string>(),
              std::string("matte_white"));
    EXPECT_NEAR(out.signature.pattern["thickness_um"].get<double>(),
                100.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["cure_temp_c"].get<double>(),
                100.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["cure_time_min"].get<double>(),
                30.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern["cure_schedule_ok"].get<bool>());
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillPaintOp, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::paint_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.paint_type   = "wet";
    in.color        = "blue";
    in.thickness_um = 60.0;
    auto out = skill::paint_op::apply(*stock, in);

    auto cands = skill::paint_op::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["color"].get<std::string>(),
              std::string("blue"));

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::paint_op::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty());
}

// ─── 6. Defaults produce sane signature ──────────────────────────────────
TEST(SkillPaintOp, DefaultsAreSane)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::paint_op::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    // all other fields default

    auto out = skill::paint_op::apply(*stock, in);
    EXPECT_NEAR(out.signature.params["thickness_um"].get<double>(), 50.0, 1e-6);
    EXPECT_EQ(out.signature.params["paint_type"].get<std::string>(),
              std::string("wet"));
}
