// @lat: [[process/test-strategy#skill round-trip]]
//
// powder_coat skill — Slice-8 GEOMETRIC tests.  apply() fuses a real
// powder-coat film slab; volume should INCREASE by ~ (face_area × thk).

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

// ─── 1. Apply: volume INCREASES by ~ (area × thickness) ──────────────────
TEST(SkillPowderCoat, ApplyGrowsRealThinShell)
{
    // 50 × 50 = 2500 mm² × 75 μm = 187.5 mm³
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::powder_coat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.color         = "black";
    in.thickness_um  = 75.0;
    in.cure_temp_c   = 200.0;
    in.cure_time_min = 15.0;
    in.texture       = "smooth";

    auto out = skill::powder_coat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double delta    = volumeOf(out.workpiece->shape()) - volBefore;
    const double expected = 50.0 * 50.0 * (75.0 * 1.0e-3);   // 187.5 mm³
    EXPECT_GT(delta, expected * 0.8);
    EXPECT_LT(delta, expected * 1.2);
    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("powder_coat"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("electrostatic_powder_gun"));
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_GT(out.signature.tooling.extra.value("stock_added_mm3", 0.0), 0.0);
}

// ─── 2. DFM: thickness [50, 300] μm + area ≥ 1 cm² ───────────────────────
TEST(SkillPowderCoat, ValidateRejectsOutOfRangeThicknessAndTinyArea)
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

    auto tiny = skill::createCuboidStock(5.0, 5.0, 5.0);
    skill::powder_coat::Input ok;
    ok.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    ok.thickness_um  = 75.0;
    ok.cure_temp_c   = 200.0;
    ok.cure_time_min = 15.0;
    auto rTiny = skill::powder_coat::validate(*tiny, ok);
    EXPECT_FALSE(rTiny.passed);
    bool foundArea = false;
    for (const auto& f : rTiny.findings)
        if (f.code == "DFM-PWD-AREA") { foundArea = true; break; }
    EXPECT_TRUE(foundArea);
}

// ─── 3. Cure temp outside [180, 220] → info, still fuses ─────────────────
TEST(SkillPowderCoat, CureTempInfoHintStillFuses)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::powder_coat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.color         = "black";
    in.thickness_um  = 75.0;
    in.cure_temp_c   = 250.0;
    in.cure_time_min = 15.0;
    in.texture       = "smooth";

    auto r = skill::powder_coat::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PWD-CURE") { found = true; break; }
    EXPECT_TRUE(found);

    auto out = skill::powder_coat::apply(*stock, in);
    EXPECT_FALSE(out.signature.pattern["cure_schedule_ok"].get<bool>());
    EXPECT_GT(volumeOf(out.workpiece->shape()) - volBefore, 0.0);
}

// ─── 4. Signature carries geometric delta + texture ──────────────────────
TEST(SkillPowderCoat, SignatureRecordsGeometricDelta)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    const double volBefore = volumeOf(stock->shape());

    skill::powder_coat::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.color         = "champagne";
    in.thickness_um  = 120.0;
    in.cure_temp_c   = 200.0;
    in.cure_time_min = 20.0;
    in.texture       = "metallic";

    auto out = skill::powder_coat::apply(*stock, in);
    const double delta = volumeOf(out.workpiece->shape()) - volBefore;

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("powder_coat"));
    EXPECT_EQ(out.signature.pattern["texture"].get<std::string>(),
              std::string("metallic"));
    EXPECT_TRUE(out.signature.pattern["cure_schedule_ok"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
    const double slabVol = out.signature.pattern["slab_volume_mm3"].get<double>();
    EXPECT_GT(slabVol, delta * 0.8);
    EXPECT_LT(slabVol, delta * 1.2);
}

// ─── 5. Recognize: geometric primary, history fallback ───────────────────
TEST(SkillPowderCoat, RecognizeGeometricThenHistory)
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
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 0.5, 1e-6);
    EXPECT_NEAR(
        cands[0].recovered_params["thickness_um"].get<double>(),
        100.0, 100.0 * 0.2);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsRaw = skill::powder_coat::recognize(raw);
    EXPECT_FALSE(candsRaw.empty());
    EXPECT_NEAR(candsRaw[0].confidence, 0.5, 1e-6);
}
