// @lat: [[process/test-strategy#skill round-trip]]
//
// electroplate skill — Slice-8 GEOMETRIC tests.  apply() fuses a real
// plating slab; volume should INCREASE by approximately
// (target_face_area × thickness_um × 1e-3) mm³.

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

// ─── 1. Apply: volume INCREASES by ~ (area × thickness) ──────────────────
TEST(SkillElectroplate, ApplyGrowsRealThinShell)
{
    // 50 × 50 = 2500 mm² × 15 μm = 37.5 mm³
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "steel_1018");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "nickel";
    in.coating_thickness_um = 15.0;

    auto out = skill::electroplate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double delta    = volumeOf(out.workpiece->shape()) - volBefore;
    const double expected = 50.0 * 50.0 * (15.0 * 1.0e-3);    // 37.5 mm³
    EXPECT_GT(delta, expected * 0.8);
    EXPECT_LT(delta, expected * 1.2);
    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("electroplate"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("electroplate_bath"));
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_GT(out.signature.tooling.extra.value("stock_added_mm3", 0.0), 0.0);
}

// ─── 2. DFM: thickness [1, 200] μm + area ≥ 1 cm² enforced ───────────────
TEST(SkillElectroplate, ValidateRejectsOutOfRangeThicknessAndTinyArea)
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

    // Tiny face: 5 × 5 × 5 — top face 25 mm² < 100 mm²
    auto tiny = skill::createCuboidStock(5.0, 5.0, 5.0, "steel_1018");
    skill::electroplate::Input ok;
    ok.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    ok.plating_metal        = "nickel";
    ok.coating_thickness_um = 10.0;
    auto rTiny = skill::electroplate::validate(*tiny, ok);
    EXPECT_FALSE(rTiny.passed);
    bool foundArea = false;
    for (const auto& f : rTiny.findings)
        if (f.code == "DFM-EP-AREA") { foundArea = true; break; }
    EXPECT_TRUE(foundArea);
}

// ─── 3. Signature carries geometric delta + plating metadata ─────────────
TEST(SkillElectroplate, SignatureRecordsGeometricDelta)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0, "steel_1018");
    const double volBefore = volumeOf(stock->shape());

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "chrome";
    in.coating_thickness_um = 5.0;
    in.base_material        = "steel_1018";

    auto out = skill::electroplate::apply(*stock, in);
    const double delta = volumeOf(out.workpiece->shape()) - volBefore;

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("electroplate"));
    EXPECT_EQ(out.signature.pattern["plating_metal"].get<std::string>(),
              std::string("chrome"));
    EXPECT_EQ(out.signature.pattern["plating_fit"].get<std::string>(),
              std::string("good"));    // chrome on steel = good
    EXPECT_GT(out.signature.pattern["deposit_hardness_hv"].get<double>(), 500.0);
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
    const double slabVol = out.signature.pattern["slab_volume_mm3"].get<double>();
    EXPECT_GT(slabVol, delta * 0.8);
    EXPECT_LT(slabVol, delta * 1.2);
}

// ─── 4. Plating-base compatibility (Cr on Al → needs strike), still grows ─
TEST(SkillElectroplate, PlatingFitNeedsStrikeStillFuses)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "aluminum_6061");
    const double volBefore = volumeOf(stock->shape());

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "chrome";
    in.coating_thickness_um = 5.0;

    auto r = skill::electroplate::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // info, not error — apply still proceeds
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-EP-FIT") { found = true; break; }
    EXPECT_TRUE(found);

    auto out = skill::electroplate::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["plating_fit"].get<std::string>(),
              std::string("needs_strike"));
    EXPECT_GT(volumeOf(out.workpiece->shape()) - volBefore, 0.0);
}

// ─── 5. Recognize: geometric primary, history fallback ───────────────────
TEST(SkillElectroplate, RecognizeGeometricThenHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "steel_1018");

    skill::electroplate::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.plating_metal        = "zinc";
    in.coating_thickness_um = 10.0;

    auto out = skill::electroplate::apply(*stock, in);

    auto cands = skill::electroplate::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 0.5, 1e-6);
    EXPECT_NEAR(
        cands[0].recovered_params["coating_thickness_um"].get<double>(),
        10.0, 10.0 * 0.2);

    // Raw shape (no feature history) — still geometrically findable.
    skill::Workpiece raw(out.workpiece->shape(), "steel_1018");
    auto candsRaw = skill::electroplate::recognize(raw);
    EXPECT_FALSE(candsRaw.empty());
    EXPECT_NEAR(candsRaw[0].confidence, 0.5, 1e-6);
}
