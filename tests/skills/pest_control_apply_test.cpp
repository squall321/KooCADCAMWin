// @lat: [[process/test-strategy#skill round-trip]]
//
// pest_control_apply — 5-case sweep covering identity-geometry synthesis,
// DFM gates on dose / chemical_name, metadata signature replay, and
// method-driven tooling selection.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pest_control_apply.hpp"

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

skill::pest_control_apply::Input goodInput()
{
    skill::pest_control_apply::Input in;
    in.chemical_name      = "glyphosate";
    in.dose_ml_ha         = 1500.0;
    in.application_method = "spray";
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPestControlApply, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::pest_control_apply::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("pest_control_apply"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects empty chemical, zero dose, over-cap dose ──────────────
TEST(SkillPestControlApply, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto bad = goodInput();
    bad.chemical_name = "";                  // invalid
    auto r = skill::pest_control_apply::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::pest_control_apply::apply(*stock, bad), skill::SkillError);

    bad = goodInput();
    bad.dose_ml_ha = 0.0;                    // invalid
    EXPECT_FALSE(skill::pest_control_apply::validate(*stock, bad).passed);

    bad = goodInput();
    bad.dose_ml_ha = 15000.0;                // > 10000 cap
    auto r2 = skill::pest_control_apply::validate(*stock, bad);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PEST-DOSE"));
}

// ── 3. Signature records kind + chem/dose/method ─────────────────────────
TEST(SkillPestControlApply, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.chemical_name      = "imidacloprid";
    in.dose_ml_ha         = 800.0;
    in.application_method = "drench";

    auto out = skill::pest_control_apply::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pest_control_apply"));
    EXPECT_EQ(out.signature.pattern["chemical_name"].get<std::string>(),
              std::string("imidacloprid"));
    EXPECT_NEAR(out.signature.pattern["dose_ml_ha"].get<double>(), 800.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["application_method"].get<std::string>(),
              std::string("drench"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPestControlApply, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::pest_control_apply::apply(*stock, goodInput());

    auto cands = skill::pest_control_apply::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["chemical_name"].get<std::string>(),
              std::string("glyphosate"));
    EXPECT_NEAR(cands[0].recovered_params["dose_ml_ha"].get<double>(),
                1500.0, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::pest_control_apply::recognize(raw).empty());
}

// ── 5. SPECIFIC: method drives tool_type ─────────────────────────────────
TEST(SkillPestControlApply, MethodDrivesToolType)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto sprayIn = goodInput();
    sprayIn.application_method = "spray";
    auto sprayOut = skill::pest_control_apply::apply(*stock, sprayIn);
    EXPECT_EQ(sprayOut.signature.tooling.tool_type,
              std::string("boom_sprayer"));

    auto granuleIn = goodInput();
    granuleIn.application_method = "granule";
    auto granuleOut = skill::pest_control_apply::apply(*stock, granuleIn);
    EXPECT_EQ(granuleOut.signature.tooling.tool_type,
              std::string("granule_broadcaster"));

    // Unknown method → info finding but passes; pattern records "spray" fallback.
    auto unkIn = goodInput();
    unkIn.application_method = "fog";
    auto rUnk = skill::pest_control_apply::validate(*stock, unkIn);
    EXPECT_TRUE(rUnk.passed);
    EXPECT_TRUE(hasFinding(rUnk, "DFM-PEST-METHOD"));
    auto unkOut = skill::pest_control_apply::apply(*stock, unkIn);
    EXPECT_EQ(unkOut.signature.pattern["application_method"].get<std::string>(),
              std::string("spray"));
}
