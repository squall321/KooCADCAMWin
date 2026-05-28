// @lat: [[process/test-strategy#skill round-trip]]
//
// ic_bond — die wire/wedge bonding, 5-case sweep.
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput (US power > 5 W)
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. ToolMaterialPicksWireFromBondType  (specific assertion)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ic_bond.hpp"

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
TEST(SkillIcBond, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5, "alumina");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::ic_bond::Input in;
    in.bond_type          = "ball";
    in.bond_count         = 64;
    in.ultrasonic_power_w = 0.5;

    auto out = skill::ic_bond::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("ic_bond"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: invalid inputs are rejected ─────────────────────────────────
TEST(SkillIcBond, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5, "alumina");

    skill::ic_bond::Input zeroCount;
    zeroCount.bond_type          = "ball";
    zeroCount.bond_count         = 0;          // invalid
    zeroCount.ultrasonic_power_w = 0.5;
    auto r1 = skill::ic_bond::validate(*stock, zeroCount);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::ic_bond::apply(*stock, zeroCount), skill::SkillError);

    skill::ic_bond::Input tooMuchUs;
    tooMuchUs.bond_type          = "wedge";
    tooMuchUs.bond_count         = 100;
    tooMuchUs.ultrasonic_power_w = 8.0;        // > 5.0 W → die crack
    auto r2 = skill::ic_bond::validate(*stock, tooMuchUs);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BOND-USPOWER-HIGH"));
    EXPECT_THROW(skill::ic_bond::apply(*stock, tooMuchUs), skill::SkillError);

    skill::ic_bond::Input badType;
    badType.bond_type          = "rivet";       // not in catalog
    badType.bond_count         = 100;
    badType.ultrasonic_power_w = 0.5;
    auto r3 = skill::ic_bond::validate(*stock, badType);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-BOND-TYPE"));
}

// ─── 3. Signature carries kind + key parameters ──────────────────────────
TEST(SkillIcBond, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5, "alumina");

    skill::ic_bond::Input in;
    in.bond_type          = "stitch";
    in.bond_count         = 256;
    in.ultrasonic_power_w = 1.2;

    auto out = skill::ic_bond::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ic_bond"));
    EXPECT_EQ(out.signature.pattern["bond_type"].get<std::string>(),
              std::string("stitch"));
    EXPECT_EQ(out.signature.pattern["bond_count"].get<int>(), 256);
    EXPECT_NEAR(out.signature.pattern["ultrasonic_power_w"].get<double>(),
                1.2, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("wire_bonder"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillIcBond, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5, "alumina");

    skill::ic_bond::Input in;
    in.bond_type          = "ball";
    in.bond_count         = 48;
    in.ultrasonic_power_w = 0.4;

    auto out = skill::ic_bond::apply(*stock, in);

    auto cands = skill::ic_bond::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["bond_type"].get<std::string>(),
              std::string("ball"));
    EXPECT_EQ(cands[0].recovered_params["bond_count"].get<int>(), 48);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::ic_bond::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "ic_bond is not geometrically recognizable";
}

// ─── 5. Tooling wire material follows bond_type (specific assertion) ─────
TEST(SkillIcBond, ToolMaterialPicksWireFromBondType)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5, "alumina");

    skill::ic_bond::Input ballIn;
    ballIn.bond_type          = "ball";
    ballIn.bond_count         = 20;
    ballIn.ultrasonic_power_w = 0.3;
    auto outBall = skill::ic_bond::apply(*stock, ballIn);
    EXPECT_EQ(outBall.signature.tooling.tool_material,
              std::string("gold_wire"));

    skill::ic_bond::Input wedgeIn = ballIn;
    wedgeIn.bond_type = "wedge";
    auto outWedge = skill::ic_bond::apply(*stock, wedgeIn);
    EXPECT_EQ(outWedge.signature.tooling.tool_material,
              std::string("aluminum_wire"));
}
