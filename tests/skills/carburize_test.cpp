// @lat: [[process/test-strategy#skill round-trip]]
//
// carburize skill — 5-case sweep covering identity-geometry synthesis,
// case-depth + hardness DFM rejection, signature metadata, metadata
// recognition replay, and a temperature-window info-only check.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/carburize.hpp"

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

}  // namespace

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillCarburize, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1018");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::carburize::Input in;
    in.material            = "carbon_steel_1018";
    in.temperature_c       = 925.0;
    in.time_h              = 8.0;
    in.case_depth_mm       = 1.0;
    in.target_hardness_hrc = 60.0;

    auto out = skill::carburize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("carburize"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects out-of-range case depth + hardness ──────────────
TEST(SkillCarburize, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1018");

    skill::carburize::Input tooShallow;
    tooShallow.case_depth_mm = 0.01;     // < 0.05 mm
    auto r1 = skill::carburize::validate(*stock, tooShallow);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-CARBURIZE-CASE"));
    EXPECT_THROW(skill::carburize::apply(*stock, tooShallow), skill::SkillError);

    skill::carburize::Input tooDeep;
    tooDeep.case_depth_mm = 5.0;         // > 3.0 mm
    auto r2 = skill::carburize::validate(*stock, tooDeep);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CARBURIZE-CASE"));

    skill::carburize::Input badHardness;
    badHardness.target_hardness_hrc = 100.0;  // out of HRC scale
    auto r3 = skill::carburize::validate(*stock, badHardness);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-CARBURIZE-HARDNESS"));

    skill::carburize::Input zeroTime;
    zeroTime.time_h = 0.0;
    auto r4 = skill::carburize::validate(*stock, zeroTime);
    EXPECT_FALSE(r4.passed);
    EXPECT_THROW(skill::carburize::apply(*stock, zeroTime), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillCarburize, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1018");

    skill::carburize::Input in;
    in.material            = "carbon_steel_1018";
    in.temperature_c       = 900.0;
    in.time_h              = 12.0;
    in.case_depth_mm       = 1.5;
    in.target_hardness_hrc = 58.0;

    auto out = skill::carburize::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("carburize"));
    EXPECT_NEAR(out.signature.pattern["temperature_c"].get<double>(),
                900.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["time_h"].get<double>(), 12.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["case_depth_mm"].get<double>(), 1.5, 1e-6);
    EXPECT_NEAR(out.signature.pattern["target_hardness_hrc"].get<double>(),
                58.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["requires_temper"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("carburize_furnace"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCarburize, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1018");

    skill::carburize::Input in;
    in.material            = "carbon_steel_1018";
    in.temperature_c       = 925.0;
    in.time_h              = 8.0;
    in.case_depth_mm       = 1.0;
    in.target_hardness_hrc = 60.0;
    auto out = skill::carburize::apply(*stock, in);

    auto cands = skill::carburize::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["case_depth_mm"].get<double>(),
                1.0, 1e-6);

    // Stripped workpiece (no features) → recognize is empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::carburize::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "carburize cannot be recognized geometrically";
}

// ─── 5. Non-carburizable material emits info hint but proceeds ───────────
TEST(SkillCarburize, NonCarburizableMaterialIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::carburize::Input in;
    in.material      = "aluminum_6061";
    in.temperature_c = 925.0;
    in.time_h        = 8.0;
    in.case_depth_mm = 1.0;

    auto r = skill::carburize::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "non-carburizable material should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-CARBURIZE-MATERIAL"));
    EXPECT_NO_THROW(skill::carburize::apply(*stock, in));
}
