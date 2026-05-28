// @lat: [[process/test-strategy#skill round-trip]]
//
// electroplate skill — 6-case sweep covering identity-geometry synthesis,
// thickness DFM clamps, plating-base compatibility, metadata recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/electroplate.hpp"

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
TEST(SkillElectroplate, ApplyDoesNotChangeGeometry)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "steel_1018");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "nickel";
    in.coating_thickness_um = 15.0;

    auto out = skill::electroplate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("electroplate"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("electroplate_bath"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: thickness clamp at [1, 200] μm ──────────────────────────────
TEST(SkillElectroplate, ValidateRejectsOutOfRangeThickness)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "steel_1018");

    skill::electroplate::Input tooThin;
    tooThin.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooThin.plating_metal        = "nickel";
    tooThin.coating_thickness_um = 0.5;
    EXPECT_FALSE(skill::electroplate::validate(*stock, tooThin).passed);
    EXPECT_THROW(skill::electroplate::apply(*stock, tooThin), skill::SkillError);

    skill::electroplate::Input tooThick = tooThin;
    tooThick.coating_thickness_um = 300.0;
    EXPECT_FALSE(skill::electroplate::validate(*stock, tooThick).passed);
    EXPECT_THROW(skill::electroplate::apply(*stock, tooThick), skill::SkillError);
}

// ─── 3. Signature carries plating metal + thickness + fit metadata ───────
TEST(SkillElectroplate, SignatureRecordsMetadata)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0, "steel_1018");

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "chrome";
    in.coating_thickness_um = 5.0;
    in.base_material        = "steel_1018";

    auto out = skill::electroplate::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("electroplate"));
    EXPECT_EQ(out.signature.pattern["plating_metal"].get<std::string>(),
              std::string("chrome"));
    EXPECT_NEAR(out.signature.pattern["coating_thickness_um"].get<double>(),
                5.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["base_material"].get<std::string>(),
              std::string("steel_1018"));
    EXPECT_EQ(out.signature.pattern["plating_fit"].get<std::string>(),
              std::string("good"));    // chrome on steel = good
    EXPECT_GT(out.signature.pattern["deposit_hardness_hv"].get<double>(), 500.0);
}

// ─── 4. Plating-base compatibility info hint (Cr on Al → needs strike) ──
TEST(SkillElectroplate, PlatingFitNeedsStrike)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "aluminum_6061");

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "chrome";
    in.coating_thickness_um = 5.0;

    auto r = skill::electroplate::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // info, not error
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EP-FIT") { found = true; break; }
    EXPECT_TRUE(found);

    auto out = skill::electroplate::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["plating_fit"].get<std::string>(),
              std::string("needs_strike"));
}

// ─── 5. Unknown plating metal → info, but apply succeeds ─────────────────
TEST(SkillElectroplate, UnknownMetalInfoOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "steel_1018");

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "rhodium";       // not in table
    in.coating_thickness_um = 2.0;

    auto r = skill::electroplate::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EP-METAL") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::electroplate::apply(*stock, in));
}

// ─── 6. Recognize via metadata replay ────────────────────────────────────
TEST(SkillElectroplate, RecognizeFromMetadata)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "steel_1018");

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "zinc";
    in.coating_thickness_um = 10.0;
    auto out = skill::electroplate::apply(*stock, in);

    auto cands = skill::electroplate::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["plating_metal"].get<std::string>(),
              std::string("zinc"));

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::electroplate::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty());
}
