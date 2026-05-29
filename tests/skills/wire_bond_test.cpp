// @lat: [[process/test-strategy#skill round-trip]]
//
// wire_bond skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range wire diameter, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wire_bond.hpp"

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

skill::wire_bond::Input goodInput()
{
    skill::wire_bond::Input in;
    in.wire_dia_um   = 25.0;
    in.wire_material = "Au";
    in.bond_count    = 16;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillWireBond, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::wire_bond::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("wire_bond"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillWireBond, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.wire_dia_um = 5.0;             // < 15 um
    auto r = skill::wire_bond::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-WIRE-DIA"));
    EXPECT_THROW(skill::wire_bond::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.bond_count = 0;               // invalid
    auto r2 = skill::wire_bond::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillWireBond, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.wire_dia_um   = 33.0;
    in.wire_material = "Cu";
    in.bond_count    = 64;

    auto out = skill::wire_bond::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("wire_bond"));
    EXPECT_NEAR(out.signature.pattern["wire_dia_um"].get<double>(), 33.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["wire_material"].get<std::string>(),
              std::string("Cu"));
    EXPECT_EQ(out.signature.pattern["bond_count"].get<int>(), 64);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWireBond, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::wire_bond::apply(*stock, goodInput());

    auto cands = skill::wire_bond::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["wire_material"].get<std::string>(),
              std::string("Au"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::wire_bond::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "wire_bond cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: Cu wire ⇒ thermosonic_Cu_bonder tooling + bond_count ────
TEST(SkillWireBond, CopperWireSelectsThermosonicCuBonder)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto auIn = goodInput();
    auIn.wire_material = "Au";
    auto auOut = skill::wire_bond::apply(*stock, auIn);
    EXPECT_EQ(auOut.signature.tooling.tool_type,
              std::string("thermosonic_Au_bonder"));

    auto cuIn = goodInput();
    cuIn.wire_material = "Cu";
    auto cuOut = skill::wire_bond::apply(*stock, cuIn);
    EXPECT_EQ(cuOut.signature.tooling.tool_type,
              std::string("thermosonic_Cu_bonder"));

    auto alIn = goodInput();
    alIn.wire_material = "Al";
    auto alOut = skill::wire_bond::apply(*stock, alIn);
    EXPECT_EQ(alOut.signature.tooling.tool_type,
              std::string("ultrasonic_wedge_bonder"));

    // Cycle time scales linearly with bond_count.
    auto manyIn = goodInput();
    manyIn.bond_count = 64;     // 4× baseline 16
    auto manyOut = skill::wire_bond::apply(*stock, manyIn);
    EXPECT_GT(manyOut.signature.tooling.est_cycle_time_s,
              auOut.signature.tooling.est_cycle_time_s);
}
