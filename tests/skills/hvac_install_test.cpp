// @lat: [[process/test-strategy#skill round-trip]]
//
// hvac_install skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of fan_kw outside (0, 500], and metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hvac_install.hpp"

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

skill::hvac_install::Input goodInput()
{
    skill::hvac_install::Input in;
    in.duct_size_mm   = 300.0;
    in.total_length_m = 40.0;
    in.register_count = 6;
    in.fan_kw         = 5.5;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillHvacInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::hvac_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("hvac_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (fan_kw out of range) ──────────────────
TEST(SkillHvacInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.fan_kw = 0.0;         // fan_kw must be > 0
    auto r = skill::hvac_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HVAC-FAN"));
    EXPECT_THROW(skill::hvac_install::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.duct_size_mm = 5000.0;  // > 2000 mm upper limit
    auto r2 = skill::hvac_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-HVAC-DUCT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillHvacInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.duct_size_mm   = 450.0;
    in.total_length_m = 60.0;
    in.register_count = 10;
    in.fan_kw         = 11.0;

    auto out = skill::hvac_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hvac_install"));
    EXPECT_NEAR(out.signature.pattern["duct_size_mm"].get<double>(),  450.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["total_length_m"].get<double>(), 60.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["register_count"].get<int>(),     10);
    EXPECT_NEAR(out.signature.pattern["fan_kw"].get<double>(),         11.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillHvacInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::hvac_install::apply(*stock, goodInput());

    auto cands = skill::hvac_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["register_count"].get<int>(), 6);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::hvac_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "hvac_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: long throw per register emits info finding ──────────────
TEST(SkillHvacInstall, LongThrowEmitsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.total_length_m = 200.0;
    in.register_count = 4;   // 50 m / register > 30 m threshold

    auto r = skill::hvac_install::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "info-level finding must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-HVAC-COVERAGE"));

    EXPECT_NO_THROW(skill::hvac_install::apply(*stock, in));
}
