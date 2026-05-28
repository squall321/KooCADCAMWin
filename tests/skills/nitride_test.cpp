// @lat: [[process/test-strategy#skill round-trip]]
//
// nitride skill — 5-case sweep covering identity-geometry synthesis, case
// depth DFM, signature metadata, metadata recognition, and type taxonomy.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/nitride.hpp"

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
TEST(SkillNitride, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "alloy_steel_4140");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::nitride::Input in;
    in.material      = "alloy_steel_4140";
    in.temperature_c = 525.0;
    in.time_h        = 40.0;
    in.case_depth_mm = 0.3;
    in.type          = "gas";

    auto out = skill::nitride::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("nitride"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects out-of-range case depth / time ──────────────────
TEST(SkillNitride, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "alloy_steel_4140");

    skill::nitride::Input tooShallow;
    tooShallow.case_depth_mm = 0.01;     // < 0.02 mm
    auto r1 = skill::nitride::validate(*stock, tooShallow);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-NITRIDE-CASE"));
    EXPECT_THROW(skill::nitride::apply(*stock, tooShallow), skill::SkillError);

    skill::nitride::Input tooDeep;
    tooDeep.case_depth_mm = 2.0;         // > 1.0 mm
    auto r2 = skill::nitride::validate(*stock, tooDeep);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-NITRIDE-CASE"));

    skill::nitride::Input zeroTime;
    zeroTime.time_h = 0.0;
    auto r3 = skill::nitride::validate(*stock, zeroTime);
    EXPECT_FALSE(r3.passed);
    EXPECT_THROW(skill::nitride::apply(*stock, zeroTime), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillNitride, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "alloy_steel_4140");

    skill::nitride::Input in;
    in.material      = "alloy_steel_4140";
    in.temperature_c = 540.0;
    in.time_h        = 60.0;
    in.case_depth_mm = 0.5;
    in.type          = "plasma";

    auto out = skill::nitride::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("nitride"));
    EXPECT_EQ(out.signature.pattern["type"].get<std::string>(),
              std::string("plasma"));
    EXPECT_NEAR(out.signature.pattern["temperature_c"].get<double>(),
                540.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["time_h"].get<double>(), 60.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["case_depth_mm"].get<double>(),
                0.5, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["requires_temper"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("nitride_furnace"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillNitride, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "alloy_steel_4140");

    skill::nitride::Input in;
    in.material      = "alloy_steel_4140";
    in.temperature_c = 525.0;
    in.time_h        = 40.0;
    in.case_depth_mm = 0.3;
    in.type          = "salt_bath";
    auto out = skill::nitride::apply(*stock, in);

    auto cands = skill::nitride::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["type"].get<std::string>(),
              std::string("salt_bath"));

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::nitride::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "nitride cannot be recognized geometrically";
}

// ─── 5. Unknown type triggers info finding but still applies ─────────────
TEST(SkillNitride, UnknownTypeEmitsInfoOnly)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "alloy_steel_4140");

    skill::nitride::Input in;
    in.material      = "alloy_steel_4140";
    in.temperature_c = 525.0;
    in.time_h        = 40.0;
    in.case_depth_mm = 0.3;
    in.type          = "exotic_glow_discharge";  // unknown

    auto r = skill::nitride::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "unknown type should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-NITRIDE-TYPE"));
    EXPECT_NO_THROW(skill::nitride::apply(*stock, in));
}
