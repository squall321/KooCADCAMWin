// @lat: [[process/test-strategy#skill round-trip]]
//
// conveyor_transport — 5-case sweep covering identity geometry, DFM
// rejection of empty station / non-positive numbers, metadata replay,
// and transit-time arithmetic.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/conveyor_transport.hpp"

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

}  // namespace

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillConveyorTransport, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::conveyor_transport::Input in;
    in.start_station = "ST01";
    in.end_station   = "ST02";
    in.speed_m_min   = 12.0;
    in.length_m      = 3.0;

    auto out = skill::conveyor_transport::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("conveyor_transport"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty station / nonpositive numbers rejected ─────────────────
TEST(SkillConveyorTransport, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::conveyor_transport::Input noStart;
    noStart.start_station = "";
    noStart.end_station   = "ST02";
    noStart.speed_m_min   = 10.0;
    noStart.length_m      = 1.0;
    EXPECT_FALSE(skill::conveyor_transport::validate(*stock, noStart).passed);
    EXPECT_THROW(skill::conveyor_transport::apply(*stock, noStart),
                 skill::SkillError);

    skill::conveyor_transport::Input noEnd = noStart;
    noEnd.start_station = "ST01";
    noEnd.end_station   = "";
    EXPECT_FALSE(skill::conveyor_transport::validate(*stock, noEnd).passed);

    skill::conveyor_transport::Input zeroSpeed;
    zeroSpeed.start_station = "ST01";
    zeroSpeed.end_station   = "ST02";
    zeroSpeed.speed_m_min   = 0.0;
    zeroSpeed.length_m      = 1.0;
    EXPECT_FALSE(skill::conveyor_transport::validate(*stock, zeroSpeed).passed);

    skill::conveyor_transport::Input negLen = zeroSpeed;
    negLen.speed_m_min = 10.0;
    negLen.length_m    = -5.0;
    EXPECT_FALSE(skill::conveyor_transport::validate(*stock, negLen).passed);
}

// ── 3. Signature records kind + start/end/speed/length ───────────────────
TEST(SkillConveyorTransport, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::conveyor_transport::Input in;
    in.start_station = "ASSEMBLY";
    in.end_station   = "INSPECTION";
    in.speed_m_min   = 15.0;
    in.length_m      = 5.0;

    auto out = skill::conveyor_transport::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("conveyor_transport"));
    EXPECT_EQ(out.signature.pattern["start_station"].get<std::string>(),
              std::string("ASSEMBLY"));
    EXPECT_EQ(out.signature.pattern["end_station"].get<std::string>(),
              std::string("INSPECTION"));
    EXPECT_NEAR(out.signature.pattern["speed_m_min"].get<double>(), 15.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["length_m"].get<double>(),    5.0,  1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("conveyor_belt"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillConveyorTransport, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::conveyor_transport::Input in;
    in.start_station = "LOAD";
    in.end_station   = "UNLOAD";
    in.speed_m_min   = 8.0;
    in.length_m      = 2.0;

    auto out = skill::conveyor_transport::apply(*stock, in);
    auto cands = skill::conveyor_transport::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["start_station"].get<std::string>(),
              std::string("LOAD"));
    EXPECT_EQ(cands[0].recovered_params["end_station"].get<std::string>(),
              std::string("UNLOAD"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::conveyor_transport::recognize(raw).empty());
}

// ── 5. transit_time_s = (length_m / speed_m_min) * 60 ────────────────────
TEST(SkillConveyorTransport, TransitTimeIsCorrect)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::conveyor_transport::Input in;
    in.start_station = "A";
    in.end_station   = "B";
    in.speed_m_min   = 30.0;        // 30 m/min = 0.5 m/s
    in.length_m      = 6.0;         // 6 m / 0.5 m/s = 12 s

    auto out = skill::conveyor_transport::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["transit_time_s"].get<double>(),
                12.0, 1e-9);
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 12.0, 1e-9);

    // Degenerate same-station case → warning, still passes.
    skill::conveyor_transport::Input deg = in;
    deg.start_station = "X";
    deg.end_station   = "X";
    auto r = skill::conveyor_transport::validate(*stock, deg);
    EXPECT_TRUE(r.passed);
    bool warned = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-CONVEYOR-DEGENERATE" && f.severity == "warning") {
            warned = true; break;
        }
    }
    EXPECT_TRUE(warned);
}
