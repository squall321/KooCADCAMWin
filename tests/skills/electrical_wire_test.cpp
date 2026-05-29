// @lat: [[process/test-strategy#skill round-trip]]
//
// electrical_wire skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range voltage, and metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/electrical_wire.hpp"

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

skill::electrical_wire::Input goodInput()
{
    skill::electrical_wire::Input in;
    in.conductor_dia_mm = 2.6;
    in.length_m         = 20.0;
    in.voltage_v        = 230.0;
    in.circuit_count    = 1;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillElectricalWire, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::electrical_wire::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("electrical_wire"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (voltage above 35 kV) ──────────────────
TEST(SkillElectricalWire, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.voltage_v = 50000.0;   // > 35 kV MV limit
    auto r = skill::electrical_wire::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-WIRE-VOLT"));
    EXPECT_THROW(skill::electrical_wire::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.circuit_count = 0;    // must be >= 1
    auto r2 = skill::electrical_wire::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillElectricalWire, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.conductor_dia_mm = 3.6;
    in.length_m         = 35.0;
    in.voltage_v        = 400.0;
    in.circuit_count    = 3;

    auto out = skill::electrical_wire::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("electrical_wire"));
    EXPECT_NEAR(out.signature.pattern["conductor_dia_mm"].get<double>(), 3.6, 1e-9);
    EXPECT_NEAR(out.signature.pattern["length_m"].get<double>(),         35.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["voltage_v"].get<double>(),       400.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["circuit_count"].get<int>(),         3);
    EXPECT_EQ(out.signature.pattern["voltage_class"].get<std::string>(),
              std::string("LV"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillElectricalWire, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::electrical_wire::apply(*stock, goodInput());

    auto cands = skill::electrical_wire::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["voltage_v"].get<double>(),
                230.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::electrical_wire::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "electrical_wire cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: MV on thin conductor emits insulation info ──────────────
TEST(SkillElectricalWire, ThinHighVoltageEmitsInsulationInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.voltage_v        = 11000.0;   // 11 kV MV
    in.conductor_dia_mm = 4.0;       // < 5 mm threshold

    auto r = skill::electrical_wire::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "info-level finding must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-WIRE-INSUL"));

    auto out = skill::electrical_wire::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["voltage_class"].get<std::string>(),
              std::string("MV"));
}
