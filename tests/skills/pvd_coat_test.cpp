// @lat: [[process/test-strategy#skill round-trip]]
//
// pvd_coat skill — Slice-8 GEOMETRIC tests.  apply() fuses a real PVD
// ceramic slab; volume should INCREASE by ~ (face_area × thickness).
// PVD films are sub-10 μm so we widen the tolerance to ±20 % since OCCT
// Boolean precision is right at the floor of relevance.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pvd_coat.hpp"

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
TEST(SkillPvdCoat, ApplyGrowsRealThinShell)
{
    // 50 × 50 = 2500 mm² × 3 μm = 7.5 mm³
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbide_K20");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "TiN";
    in.thickness_um     = 3.0;
    in.hardness_hv      = 2200.0;

    auto out = skill::pvd_coat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double delta    = volumeOf(out.workpiece->shape()) - volBefore;
    const double expected = 50.0 * 50.0 * (3.0 * 1.0e-3);    // 7.5 mm³
    EXPECT_GT(delta, expected * 0.8);
    EXPECT_LT(delta, expected * 1.2);
    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("pvd_coat"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("pvd_chamber"));
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_GT(out.signature.tooling.extra.value("stock_added_mm3", 0.0), 0.0);
}

// ─── 2. DFM: thickness [0.5, 10] μm + area ≥ 1 cm² ───────────────────────
TEST(SkillPvdCoat, ValidateRejectsOutOfRangeThicknessAndTinyArea)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::pvd_coat::Input tooThin;
    tooThin.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooThin.coating_compound = "TiN";
    tooThin.thickness_um     = 0.2;
    EXPECT_FALSE(skill::pvd_coat::validate(*stock, tooThin).passed);
    EXPECT_THROW(skill::pvd_coat::apply(*stock, tooThin), skill::SkillError);

    skill::pvd_coat::Input tooThick = tooThin;
    tooThick.thickness_um = 25.0;
    EXPECT_FALSE(skill::pvd_coat::validate(*stock, tooThick).passed);
    EXPECT_THROW(skill::pvd_coat::apply(*stock, tooThick), skill::SkillError);

    auto tiny = skill::createCuboidStock(5.0, 5.0, 5.0);
    skill::pvd_coat::Input ok;
    ok.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    ok.coating_compound = "TiN";
    ok.thickness_um     = 3.0;
    auto rTiny = skill::pvd_coat::validate(*tiny, ok);
    EXPECT_FALSE(rTiny.passed);
    bool foundArea = false;
    for (const auto& f : rTiny.findings)
        if (f.code == "DFM-PVD-AREA") { foundArea = true; break; }
    EXPECT_TRUE(foundArea);
}

// ─── 3. Unknown compound → info, still fuses ─────────────────────────────
TEST(SkillPvdCoat, UnknownCompoundInfoStillFuses)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "ZrN";
    in.thickness_um     = 2.0;
    in.hardness_hv      = 2500.0;

    auto r = skill::pvd_coat::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PVD-COMPOUND") { found = true; break; }
    EXPECT_TRUE(found);

    auto out = skill::pvd_coat::apply(*stock, in);
    EXPECT_GT(volumeOf(out.workpiece->shape()) - volBefore, 0.0);
}

// ─── 4. Signature carries geometric delta + compound metadata ────────────
TEST(SkillPvdCoat, SignatureRecordsGeometricDelta)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);
    const double volBefore = volumeOf(stock->shape());

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "DLC";
    in.thickness_um     = 2.0;
    // hardness left at 0 → expect to be filled with compound midpoint

    auto out = skill::pvd_coat::apply(*stock, in);
    const double delta = volumeOf(out.workpiece->shape()) - volBefore;

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pvd_coat"));
    EXPECT_EQ(out.signature.pattern["coating_compound"].get<std::string>(),
              std::string("DLC"));
    // DLC midpoint = (1500 + 4000) / 2 = 2750 HV
    EXPECT_NEAR(out.signature.pattern["hardness_hv"].get<double>(),
                2750.0, 1.0);
    EXPECT_TRUE(out.signature.pattern.contains("color_rgb_hint"));
    EXPECT_TRUE(out.signature.pattern["hardness_in_compound_range"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
    const double slabVol = out.signature.pattern["slab_volume_mm3"].get<double>();
    EXPECT_GT(slabVol, delta * 0.8);
    EXPECT_LT(slabVol, delta * 1.2);
}

// ─── 5. Recognize: geometric primary, history fallback ───────────────────
TEST(SkillPvdCoat, RecognizeGeometricThenHistory)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::pvd_coat::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.coating_compound = "TiAlN";
    in.thickness_um     = 4.0;
    in.hardness_hv      = 3200.0;

    auto out = skill::pvd_coat::apply(*stock, in);

    auto cands = skill::pvd_coat::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 0.5, 1e-6);
    EXPECT_NEAR(
        cands[0].recovered_params["thickness_um"].get<double>(),
        4.0, 4.0 * 0.2);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsRaw = skill::pvd_coat::recognize(raw);
    EXPECT_FALSE(candsRaw.empty());
    EXPECT_NEAR(candsRaw[0].confidence, 0.5, 1e-6);
}
