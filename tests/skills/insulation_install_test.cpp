// @lat: [[process/test-strategy#skill round-trip]]
//
// insulation_install skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of bad thickness, and metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/insulation_install.hpp"

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

skill::insulation_install::Input goodInput()
{
    skill::insulation_install::Input in;
    in.insulation_type = "mineral_wool";
    in.thickness_mm    = 100.0;
    in.area_m2         = 50.0;
    in.r_value         = 2.5;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillInsulationInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::insulation_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("insulation_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (zero thickness / negative r) ──────────
TEST(SkillInsulationInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.thickness_mm = 0.0;   // must be > 0
    auto r = skill::insulation_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INSUL-THK"));
    EXPECT_THROW(skill::insulation_install::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.r_value = -1.0;      // must be > 0
    auto r2 = skill::insulation_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params + derived u_value ─────────────
TEST(SkillInsulationInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.insulation_type = "pir";
    in.thickness_mm    = 80.0;
    in.area_m2         = 75.0;
    in.r_value         = 4.0;

    auto out = skill::insulation_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("insulation_install"));
    EXPECT_EQ(out.signature.pattern["insulation_type"].get<std::string>(),
              std::string("pir"));
    EXPECT_NEAR(out.signature.pattern["thickness_mm"].get<double>(), 80.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["area_m2"].get<double>(),      75.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["r_value"].get<double>(),       4.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["u_value"].get<double>(),       0.25, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillInsulationInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::insulation_install::apply(*stock, goodInput());

    auto cands = skill::insulation_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["insulation_type"].get<std::string>(),
              std::string("mineral_wool"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::insulation_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "insulation_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: sub-code R-value emits info and flips thermal_class ─────
TEST(SkillInsulationInstall, SubCodeRValueEmitsInfoAndFlipsThermalClass)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.r_value = 0.5;        // < 1.0 threshold

    auto r = skill::insulation_install::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "info-level finding must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-INSUL-PERF"));

    auto out = skill::insulation_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["thermal_class"].get<std::string>(),
              std::string("sub_code"));
}
