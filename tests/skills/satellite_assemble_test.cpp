// @lat: [[process/test-strategy#skill round-trip]]
//
// satellite_assemble skill — clean-room integration no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/satellite_assemble.hpp"

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

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillSatelliteAssemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 1500.0,
                                          "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::satellite_assemble::Input in;
    in.bus_id           = "BUS-COMSAT-1";
    in.mass_kg          = 1200.0;
    in.instrument_count = 4;

    auto out = skill::satellite_assemble::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("satellite_assemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: empty bus_id + zero instruments → errors ─
TEST(SkillSatelliteAssemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 1500.0);

    skill::satellite_assemble::Input in;
    in.bus_id           = "";          // missing identifier
    in.mass_kg          = 500.0;
    in.instrument_count = 0;           // < 1

    auto r = skill::satellite_assemble::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_TRUE(hasFinding(r, "DFM-SPACE-PAYLOAD"));
    EXPECT_THROW(skill::satellite_assemble::apply(*stock, in), skill::SkillError);

    // mass too large
    skill::satellite_assemble::Input in2;
    in2.bus_id           = "BUS-X";
    in2.mass_kg          = 30000.0;    // > 25 t envelope
    in2.instrument_count = 2;
    auto r2 = skill::satellite_assemble::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SPACE-MASS"));
}

// ─── 3. Signature records kind + bus_id + mass + instrument_count ────────
TEST(SkillSatelliteAssemble, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 1500.0);

    skill::satellite_assemble::Input in;
    in.bus_id           = "BUS-EOSAT-9";
    in.mass_kg          = 850.0;
    in.instrument_count = 5;

    auto out = skill::satellite_assemble::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("satellite_assemble"));
    EXPECT_EQ(out.signature.pattern["bus_id"].get<std::string>(),
              std::string("BUS-EOSAT-9"));
    EXPECT_NEAR(out.signature.pattern["mass_kg"].get<double>(), 850.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["instrument_count"].get<int>(), 5);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSatelliteAssemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 1500.0);

    skill::satellite_assemble::Input in;
    in.bus_id           = "BUS-SAR-3";
    in.mass_kg          = 2200.0;
    in.instrument_count = 2;

    auto out  = skill::satellite_assemble::apply(*stock, in);
    auto cands = skill::satellite_assemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["bus_id"].get<std::string>(),
              std::string("BUS-SAR-3"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::satellite_assemble::recognize(raw).empty())
        << "satellite_assemble cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: mass_per_instrument derived correctly ──────────────────
TEST(SkillSatelliteAssemble, MassPerInstrumentDerived)
{
    auto stock = skill::createCuboidStock(1000.0, 1000.0, 1500.0);

    skill::satellite_assemble::Input in;
    in.bus_id           = "BUS-DIV";
    in.mass_kg          = 1000.0;
    in.instrument_count = 4;

    auto out = skill::satellite_assemble::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["mass_per_instrument_kg"].get<double>(),
                250.0, 1e-6);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("clean_room_fixture"));
    // Integration time scales with instrument count.
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}
