// @lat: [[process/test-strategy#skill round-trip]]
//
// charging_port_install skill — 5-case sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/charging_port_install.hpp"

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

skill::charging_port_install::Input goodInput()
{
    skill::charging_port_install::Input in;
    in.port_type = "CCS";
    in.max_kw    = 150.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillChargingPortInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::charging_port_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("charging_port_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillChargingPortInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);

    auto in = goodInput();
    in.port_type = "USB-C";              // invalid for EV
    auto r = skill::charging_port_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PORT-TYPE"));
    EXPECT_THROW(skill::charging_port_install::apply(*stock, in),
                 skill::SkillError);

    // Type 2 (AC) ceiling is 43 kW — 100 kW should reject.
    auto in2 = goodInput();
    in2.port_type = "Type2";
    in2.max_kw    = 100.0;
    auto r2 = skill::charging_port_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-PORT-KW"));

    auto in3 = goodInput();
    in3.max_kw = 0.0;
    auto r3 = skill::charging_port_install::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillChargingPortInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);

    auto in = goodInput();
    in.port_type = "NACS";
    in.max_kw    = 250.0;

    auto out = skill::charging_port_install::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("charging_port_install"));
    EXPECT_EQ(out.signature.pattern["port_type"].get<std::string>(),
              std::string("NACS"));
    EXPECT_NEAR(out.signature.pattern["max_kw"].get<double>(), 250.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["standard_ceiling"].get<double>(),
                250.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillChargingPortInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);
    auto out = skill::charging_port_install::apply(*stock, goodInput());

    auto cands = skill::charging_port_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["port_type"].get<std::string>(),
              std::string("CCS"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::charging_port_install::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. SPECIFIC: Type2 is AC, the others are DC ──────────────────────────
TEST(SkillChargingPortInstall, CurrentTypeDependsOnPort)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 30.0);

    auto inAc = goodInput();
    inAc.port_type = "Type2";
    inAc.max_kw    = 22.0;
    auto outAc = skill::charging_port_install::apply(*stock, inAc);
    EXPECT_EQ(outAc.signature.pattern["current_type"].get<std::string>(),
              std::string("AC"));

    auto inDc = goodInput();
    inDc.port_type = "CHAdeMO";
    inDc.max_kw    = 50.0;
    auto outDc = skill::charging_port_install::apply(*stock, inDc);
    EXPECT_EQ(outDc.signature.pattern["current_type"].get<std::string>(),
              std::string("DC"));
}
