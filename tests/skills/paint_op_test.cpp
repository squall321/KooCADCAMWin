// @lat: [[process/test-strategy#skill round-trip]]
//
// paint_op skill — Slice-8 GEOMETRIC tests.  apply() fuses a real paint
// film slab; volume should INCREASE by ~ (face_area × thickness).

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

// ─── 1. Apply: volume INCREASES by ~ (area × thickness) ──────────────────
TEST(SkillPaintOp, ApplyGrowsRealThinShell)
{
    // 50 × 50 = 2500 mm² × 50 μm = 125 mm³
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();

    skill::paint_op::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.paint_type    = "wet";
    in.color         = "red";
    in.thickness_um  = 50.0;
    in.cure_temp_c   = 80.0;
    in.cure_time_min = 20.0;

    auto out = skill::paint_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double delta    = volumeOf(out.workpiece->shape()) - volBefore;
    const double expected = 50.0 * 50.0 * (50.0 * 1.0e-3);   // 125 mm³
    EXPECT_GT(delta, expected * 0.8);
    EXPECT_LT(delta, expected * 1.2);
    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("paint_op"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("spray_gun"));
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_GT(out.signature.tooling.extra.value("stock_added_mm3", 0.0), 0.0);
}

// ─── 2. DFM: thickness [10, 500] μm + area ≥ 1 cm² ───────────────────────
TEST(SkillPaintOp, ValidateRejectsOutOfRangeThicknessAndTinyArea)
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

    auto tiny = skill::createCuboidStock(5.0, 5.0, 5.0);
    skill::paint_op::Input ok;
    ok.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    ok.thickness_um = 50.0;
    auto rTiny = skill::paint_op::validate(*tiny, ok);
    EXPECT_FALSE(rTiny.passed);
    bool foundArea = false;
    for (const auto& f : rTiny.findings)
        if (f.code == "DFM-PAINT-AREA") { foundArea = true; break; }
    EXPECT_TRUE(foundArea);
}

// ─── 3. paint_type='powder' emits warning, still fuses ───────────────────
TEST(SkillPaintOp, PowderTypeWarningStillFuses)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::paint_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.paint_type   = "powder";
    in.color        = "black";
    in.thickness_um = 75.0;

    auto r = skill::paint_op::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PAINT-TYPE" && f.severity == "warning") {
            found = true; break;
        }
    EXPECT_TRUE(found);

    auto out = skill::paint_op::apply(*stock, in);
    EXPECT_GT(volumeOf(out.workpiece->shape()) - volBefore, 0.0);
}

// ─── 4. Signature carries geometric delta + cure schedule ────────────────
TEST(SkillPaintOp, SignatureRecordsGeometricDelta)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    const double volBefore = volumeOf(stock->shape());

    skill::paint_op::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.paint_type    = "wet";
    in.color         = "matte_white";
    in.thickness_um  = 100.0;
    in.cure_temp_c   = 100.0;
    in.cure_time_min = 30.0;

    auto out = skill::paint_op::apply(*stock, in);
    const double delta = volumeOf(out.workpiece->shape()) - volBefore;

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("paint_op"));
    EXPECT_EQ(out.signature.pattern["color"].get<std::string>(),
              std::string("matte_white"));
    EXPECT_TRUE(out.signature.pattern["cure_schedule_ok"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
    const double slabVol = out.signature.pattern["slab_volume_mm3"].get<double>();
    EXPECT_GT(slabVol, delta * 0.8);
    EXPECT_LT(slabVol, delta * 1.2);
}

// ─── 5. Recognize: geometric primary, history fallback ───────────────────
TEST(SkillPaintOp, RecognizeGeometricThenHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::paint_op::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.paint_type   = "wet";
    in.color        = "blue";
    in.thickness_um = 60.0;

    auto out = skill::paint_op::apply(*stock, in);

    auto cands = skill::paint_op::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 0.5, 1e-6);
    EXPECT_NEAR(
        cands[0].recovered_params["thickness_um"].get<double>(),
        60.0, 60.0 * 0.2);

    // Raw shape — still geometrically findable.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsRaw = skill::paint_op::recognize(raw);
    EXPECT_FALSE(candsRaw.empty());
    EXPECT_NEAR(candsRaw[0].confidence, 0.5, 1e-6);
}
