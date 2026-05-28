// @lat: [[process/test-strategy#skill round-trip]]
//
// powder_coat skill — 6-case sweep covering identity-geometry synthesis,
// thickness clamps, cure-temp info, texture options, metadata recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/powder_coat.hpp"

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
TEST(SkillPowderCoat, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::powder_coat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.color         = "black";
    in.thickness_um  = 75.0;
    in.cure_temp_c   = 200.0;
    in.cure_time_min = 15.0;
    in.texture       = "smooth";

    auto out = skill::powder_coat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);

    EXPECT_EQ(out.signature.skill_id, std::string("powder_coat"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("electrostatic_powder_gun"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: thickness [50, 300] μm enforced ─────────────────────────────
TEST(SkillPowderCoat, ValidateRejectsOutOfRangeThickness)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::powder_coat::Input tooThin;
    tooThin.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooThin.thickness_um = 30.0;
    EXPECT_FALSE(skill::powder_coat::validate(*stock, tooThin).passed);
    EXPECT_THROW(skill::powder_coat::apply(*stock, tooThin), skill::SkillError);

    skill::powder_coat::Input tooThick = tooThin;
    tooThick.thickness_um = 400.0;
    EXPECT_FALSE(skill::powder_coat::validate(*stock, tooThick).passed);
    EXPECT_THROW(skill::powder_coat::apply(*stock, tooThick), skill::SkillError);
}

// ─── 3. Cure temp outside [180, 220] → info ──────────────────────────────
TEST(SkillPowderCoat, CureTempInfoHint)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::powder_coat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.color         = "black";
    in.thickness_um  = 75.0;
    in.cure_temp_c   = 250.0;   // out of typical band
    in.cure_time_min = 15.0;
    in.texture       = "smooth";

    auto r = skill::powder_coat::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // info, not error
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PWD-CURE") { found = true; break; }
    EXPECT_TRUE(found);

    auto out = skill::powder_coat::apply(*stock, in);
    EXPECT_FALSE(out.signature.pattern["cure_schedule_ok"].get<bool>());
}

// ─── 4. Signature carries texture metadata ───────────────────────────────
TEST(SkillPowderCoat, SignatureRecordsMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::powder_coat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.color         = "champagne";
    in.thickness_um  = 120.0;
    in.cure_temp_c   = 200.0;
    in.cure_time_min = 20.0;
    in.texture       = "metallic";

    auto out = skill::powder_coat::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("powder_coat"));
    EXPECT_EQ(out.signature.pattern["color"].get<std::string>(),
              std::string("champagne"));
    EXPECT_NEAR(out.signature.pattern["thickness_um"].get<double>(),
                120.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["texture"].get<std::string>(),
              std::string("metallic"));
    EXPECT_TRUE(out.signature.pattern["cure_schedule_ok"].get<bool>());
}

// ─── 5. Unknown texture → info ───────────────────────────────────────────
TEST(SkillPowderCoat, UnknownTextureInfoOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::powder_coat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.color         = "black";
    in.thickness_um  = 75.0;
    in.cure_temp_c   = 200.0;
    in.cure_time_min = 15.0;
    in.texture       = "hammered";   // not in table

    auto r = skill::powder_coat::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PWD-TEXTURE") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::powder_coat::apply(*stock, in));
}

// ─── 6. Recognize via metadata replay ────────────────────────────────────
TEST(SkillPowderCoat, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::powder_coat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.color         = "matte_black";
    in.thickness_um  = 100.0;
    in.cure_temp_c   = 200.0;
    in.cure_time_min = 15.0;
    in.texture       = "matte";

    auto out = skill::powder_coat::apply(*stock, in);

    auto cands = skill::powder_coat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["texture"].get<std::string>(),
              std::string("matte"));

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::powder_coat::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty());
}
