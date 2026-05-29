// @lat: [[process/test-strategy#skill round-trip]]
//
// dewar_assemble — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of bad jacket thickness, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/dewar_assemble.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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

skill::dewar_assemble::Input goodInput()
{
    skill::dewar_assemble::Input in;
    in.inner_dia_mm               = 200.0;
    in.vacuum_jacket_thickness_mm = 25.0;
    in.insulation_type            = "MLI";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillDewarAssemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::dewar_assemble::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("dewar_assemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (jacket too thin) ──────────────────────
TEST(SkillDewarAssemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.vacuum_jacket_thickness_mm = 2.0;   // < 5 mm minimum
    auto r = skill::dewar_assemble::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-DEWAR-GAP"));
    EXPECT_THROW(skill::dewar_assemble::apply(*stock, in), skill::SkillError);

    // jacket too thick
    auto in2 = goodInput();
    in2.vacuum_jacket_thickness_mm = 300.0;   // > 200 mm
    auto r2 = skill::dewar_assemble::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-DEWAR-GAP"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillDewarAssemble, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.inner_dia_mm               = 350.0;
    in.vacuum_jacket_thickness_mm = 50.0;
    in.insulation_type            = "aerogel";

    auto out = skill::dewar_assemble::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("dewar_assemble"));
    EXPECT_NEAR(out.signature.pattern["inner_dia_mm"].get<double>(),
                350.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["vacuum_jacket_thickness_mm"].get<double>(),
                50.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["insulation_type"].get<std::string>(),
              std::string("aerogel"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillDewarAssemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::dewar_assemble::apply(*stock, goodInput());

    auto cands = skill::dewar_assemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["insulation_type"].get<std::string>(),
              std::string("MLI"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::dewar_assemble::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "dewar_assemble cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: MLI < aerogel < perlite < none for heat leak ────────────
TEST(SkillDewarAssemble, MLIBeatsAerogelBeatsPerliteBeatsNone)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto mliIn = goodInput();      mliIn.insulation_type = "MLI";
    auto aeIn  = goodInput();      aeIn.insulation_type  = "aerogel";
    auto pIn   = goodInput();      pIn.insulation_type   = "perlite";
    auto nIn   = goodInput();      nIn.insulation_type   = "none";

    const double mli = skill::dewar_assemble::apply(*stock, mliIn)
        .signature.pattern["estimated_heat_leak_w_per_m2"].get<double>();
    const double ae = skill::dewar_assemble::apply(*stock, aeIn)
        .signature.pattern["estimated_heat_leak_w_per_m2"].get<double>();
    const double p = skill::dewar_assemble::apply(*stock, pIn)
        .signature.pattern["estimated_heat_leak_w_per_m2"].get<double>();
    const double n = skill::dewar_assemble::apply(*stock, nIn)
        .signature.pattern["estimated_heat_leak_w_per_m2"].get<double>();

    EXPECT_LT(mli, ae);
    EXPECT_LT(ae,  p);
    EXPECT_LT(p,   n);
}
