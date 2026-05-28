// @lat: [[process/test-strategy#skill round-trip]]
//
// wave_solder — through-hole wave-solder skill, 5-case sweep covering
// process-window gates.
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput (out-of-window temp_c)
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. DwellTimeDerivedFromConveyorSpeed (specific assertion)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wave_solder.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

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
TEST(SkillWaveSolder, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 1.6, "FR4");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::wave_solder::Input in;
    in.solder_alloy   = "SAC305";
    in.temp_c         = 260.0;
    in.conveyor_speed = 1.0;

    auto out = skill::wave_solder::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("wave_solder"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: temp_c outside [230, 290] is an ERROR ───────────────────────
TEST(SkillWaveSolder, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 1.6, "FR4");

    skill::wave_solder::Input tooCold;
    tooCold.solder_alloy   = "SAC305";
    tooCold.temp_c         = 200.0;          // below solidus
    tooCold.conveyor_speed = 1.0;
    auto r1 = skill::wave_solder::validate(*stock, tooCold);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-WAVE-TEMP-LOW"));
    EXPECT_THROW(skill::wave_solder::apply(*stock, tooCold), skill::SkillError);

    skill::wave_solder::Input tooHot = tooCold;
    tooHot.temp_c         = 320.0;           // FR-4 delamination
    auto r2 = skill::wave_solder::validate(*stock, tooHot);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-WAVE-TEMP-HIGH"));

    // Zero conveyor_speed → error.
    skill::wave_solder::Input stopped = tooCold;
    stopped.temp_c         = 260.0;
    stopped.conveyor_speed = 0.0;
    auto r3 = skill::wave_solder::validate(*stock, stopped);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature carries kind + key parameters ──────────────────────────
TEST(SkillWaveSolder, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 1.6, "FR4");

    skill::wave_solder::Input in;
    in.solder_alloy   = "Sn63Pb37";
    in.temp_c         = 250.0;
    in.conveyor_speed = 1.2;

    auto out = skill::wave_solder::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("wave_solder"));
    EXPECT_EQ(out.signature.pattern["solder_alloy"].get<std::string>(),
              std::string("Sn63Pb37"));
    EXPECT_NEAR(out.signature.pattern["temp_c"].get<double>(),         250.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["conveyor_speed"].get<double>(),   1.2, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("wave_solder_pot"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWaveSolder, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 1.6, "FR4");

    skill::wave_solder::Input in;
    in.solder_alloy   = "SAC305";
    in.temp_c         = 260.0;
    in.conveyor_speed = 1.0;

    auto out = skill::wave_solder::apply(*stock, in);

    auto cands = skill::wave_solder::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["solder_alloy"].get<std::string>(),
              std::string("SAC305"));

    // Strip features → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::wave_solder::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "wave_solder is not geometrically recognizable";
}

// ─── 5. Dwell time derives from effective wave width / conveyor speed ────
TEST(SkillWaveSolder, DwellTimeDerivedFromConveyorSpeed)
{
    auto stock = skill::createCuboidStock(120.0, 80.0, 1.6, "FR4");

    // 0.025 m / (1.5 m/min) × 60 s/min = 1.0 s
    skill::wave_solder::Input in;
    in.solder_alloy   = "SAC305";
    in.temp_c         = 260.0;
    in.conveyor_speed = 1.5;

    auto out = skill::wave_solder::apply(*stock, in);

    const double expected = (0.025 / 1.5) * 60.0;
    EXPECT_NEAR(out.signature.pattern["dwell_time_s"].get<double>(),
                expected, 1e-6);
}
