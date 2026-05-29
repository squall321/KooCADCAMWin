// @lat: [[process/test-strategy#skill round-trip]]
//
// pellet_extrude skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of bad throughput, signature metadata, metadata-only
// recognition, and specific-energy estimate.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pellet_extrude.hpp"

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

skill::pellet_extrude::Input goodInput()
{
    skill::pellet_extrude::Input in;
    in.polymer         = "ABS";
    in.throughput_kg_h = 120.0;
    in.screw_speed_rpm = 250.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPelletExtrude, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::pellet_extrude::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("pellet_extrude"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (throughput 0, then >2000) ─────────────
TEST(SkillPelletExtrude, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.throughput_kg_h = 0.0;
    auto r = skill::pellet_extrude::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-EXT-THROUGHPUT"));
    EXPECT_THROW(skill::pellet_extrude::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.screw_speed_rpm = 1200.0;
    auto r2 = skill::pellet_extrude::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-EXT-SCREW"));

    auto in3 = goodInput();
    in3.polymer = "";
    auto r3 = skill::pellet_extrude::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + polymer + throughput + rpm ───────────────
TEST(SkillPelletExtrude, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.polymer         = "PA66";
    in.throughput_kg_h = 300.0;
    in.screw_speed_rpm = 350.0;

    auto out = skill::pellet_extrude::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pellet_extrude"));
    EXPECT_EQ(out.signature.pattern["polymer"].get<std::string>(),
              std::string("PA66"));
    EXPECT_NEAR(out.signature.pattern["throughput_kg_h"].get<double>(),
                300.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["screw_speed_rpm"].get<double>(),
                350.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPelletExtrude, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::pellet_extrude::apply(*stock, goodInput());

    auto cands = skill::pellet_extrude::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["polymer"].get<std::string>(),
              std::string("ABS"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::pellet_extrude::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "pellet_extrude cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: specific_energy_kwh_kg scales with screw speed ──────────
TEST(SkillPelletExtrude, SpecificEnergyScalesWithScrewSpeed)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto slow = goodInput();
    slow.screw_speed_rpm = 100.0;
    auto slowOut = skill::pellet_extrude::apply(*stock, slow);

    auto fast = goodInput();
    fast.screw_speed_rpm = 500.0;
    auto fastOut = skill::pellet_extrude::apply(*stock, fast);

    const double eSlow =
        slowOut.signature.pattern["specific_energy_kwh_kg"].get<double>();
    const double eFast =
        fastOut.signature.pattern["specific_energy_kwh_kg"].get<double>();

    EXPECT_GT(eFast, eSlow)
        << "higher screw rpm → higher specific energy (more shear input)";
    EXPECT_EQ(slowOut.signature.tooling.tool_type,
              std::string("single_screw_extruder"));
}
