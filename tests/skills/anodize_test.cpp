// @lat: [[process/test-strategy#skill round-trip]]
//
// anodize skill — Slice-8 GEOMETRIC tests.  apply() now grows a real
// oxide-layer slab via fuse, so volume should INCREASE by approximately
// (target_face_area × thickness_um × 1e-3) mm³.  Recognize() must match
// the resulting parallel face-pair pattern at confidence 0.5.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/anodize.hpp"

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
TEST(SkillAnodize, ApplyGrowsRealThinShell)
{
    // 50 × 50 = 2500 mm² top face × 25 μm = 62.5 mm³ added (nominal).
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 25.0;
    in.color                = "black";
    in.seal_type            = "hot_water";

    auto out = skill::anodize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double delta    = volAfter - volBefore;
    const double expected = 50.0 * 50.0 * (25.0 * 1.0e-3);     // 62.5 mm³
    EXPECT_GT(delta, expected * 0.8);   // within ±20 %
    EXPECT_LT(delta, expected * 1.2);
    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("anodize"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("anodize_bath"));
    // Additive convention: stock_removed_mm3 < 0 (volume ADDED).
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_GT(out.signature.tooling.extra.value("stock_added_mm3", 0.0), 0.0);
}

// ─── 2. DFM: thickness < 5 μm or > 75 μm rejected, area < 1 cm² rejected ─
TEST(SkillAnodize, ValidateRejectsOutOfRangeThicknessAndTinyArea)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::anodize::Input tooThin;
    tooThin.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooThin.coating_thickness_um = 2.0;
    tooThin.color                = "natural";
    auto r1 = skill::anodize::validate(*stock, tooThin);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::anodize::apply(*stock, tooThin), skill::SkillError);

    skill::anodize::Input tooThick = tooThin;
    tooThick.coating_thickness_um = 120.0;
    EXPECT_FALSE(skill::anodize::validate(*stock, tooThick).passed);
    EXPECT_THROW(skill::anodize::apply(*stock, tooThick), skill::SkillError);

    bool foundThk = false;
    for (const auto& f : r1.findings)
        if (f.code == "DFM-ANODIZE-THK") { foundThk = true; break; }
    EXPECT_TRUE(foundThk);

    // Tiny face area: 5 × 5 × 5 = 25 mm² per face → < 1 cm² floor
    auto tiny = skill::createCuboidStock(5.0, 5.0, 5.0);
    skill::anodize::Input ok;
    ok.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    ok.coating_thickness_um = 25.0;
    ok.color                = "natural";
    auto rTiny = skill::anodize::validate(*tiny, ok);
    EXPECT_FALSE(rTiny.passed);
    bool foundArea = false;
    for (const auto& f : rTiny.findings)
        if (f.code == "DFM-ANODIZE-AREA") { foundArea = true; break; }
    EXPECT_TRUE(foundArea);
}

// ─── 3. Signature pattern carries geometric volume + face area ───────────
TEST(SkillAnodize, SignatureRecordsGeometricDelta)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    const double volBefore = volumeOf(stock->shape());

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 60.0;     // Type III hard
    in.color                = "gold";
    in.seal_type            = "nickel_acetate";

    auto out = skill::anodize::apply(*stock, in);
    const double delta = volumeOf(out.workpiece->shape()) - volBefore;

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("anodize"));
    EXPECT_EQ(out.signature.pattern["color"].get<std::string>(),
              std::string("gold"));
    EXPECT_EQ(out.signature.pattern["anodize_class"].get<std::string>(),
              std::string("Type_III_hard"));
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
    EXPECT_TRUE(out.signature.pattern.contains("slab_volume_mm3"));
    EXPECT_GT(out.signature.pattern["target_face_area_mm2"].get<double>(),
              1000.0);   // 40 × 40 = 1600
    // Slab volume ~ delta (within fuse tolerance)
    const double slabVol = out.signature.pattern["slab_volume_mm3"].get<double>();
    EXPECT_GT(slabVol, delta * 0.8);
    EXPECT_LT(slabVol, delta * 1.2);
}

// ─── 4. Non-aluminum substrate emits info hint but does not block apply ──
TEST(SkillAnodize, NonAluminumInfoHint)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "steel_1018");

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 25.0;
    in.color                = "black";

    auto r = skill::anodize::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // info, not error
    bool foundMat = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ANODIZE-MATERIAL") { foundMat = true; break; }
    EXPECT_TRUE(foundMat);
    EXPECT_NO_THROW(skill::anodize::apply(*stock, in));
}

// ─── 5. Recognize: geometric primary @ confidence 0.5, history fallback ──
TEST(SkillAnodize, RecognizeGeometricThenHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::anodize::Input in;
    in.entry_face           = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_thickness_um = 25.0;
    in.color                = "red";
    in.seal_type            = "hot_water";

    auto out = skill::anodize::apply(*stock, in);

    // Result wp carries history — recognize emits geometric candidate first,
    // which yields confidence 0.5 (history fallback is only when geometric
    // returns empty).
    auto cands = skill::anodize::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 0.5, 1e-6);
    EXPECT_NEAR(
        cands[0].recovered_params["coating_thickness_um"].get<double>(),
        25.0, 25.0 * 0.2);                                   // ±20 %

    // Strip features but retain the geometry → still geometrically findable.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsRaw = skill::anodize::recognize(raw);
    EXPECT_FALSE(candsRaw.empty());
    EXPECT_NEAR(candsRaw[0].confidence, 0.5, 1e-6);
}
