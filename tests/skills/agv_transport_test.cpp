// @lat: [[process/test-strategy#skill round-trip]]
//
// agv_transport — 5-case sweep covering identity-geometry synthesis,
// route + speed DFM gates, metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/agv_transport.hpp"

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
TEST(SkillAgvTransport, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::agv_transport::Input in;
    in.origin      = "M-01";
    in.destination = "A-04";
    in.distance_m  = 50.0;
    in.speed_m_s   = 1.2;

    auto out = skill::agv_transport::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("agv_transport"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty origin/dest, zero distance/speed rejected ──────────────
TEST(SkillAgvTransport, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::agv_transport::Input bad;
    bad.origin      = "";              // invalid
    bad.destination = "A-01";
    bad.distance_m  = 10.0;
    bad.speed_m_s   = 1.0;
    auto r = skill::agv_transport::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::agv_transport::apply(*stock, bad), skill::SkillError);

    bad.origin      = "M-01";
    bad.destination = "";              // invalid
    EXPECT_FALSE(skill::agv_transport::validate(*stock, bad).passed);

    bad.destination = "A-01";
    bad.distance_m  = 0.0;             // invalid
    EXPECT_FALSE(skill::agv_transport::validate(*stock, bad).passed);

    bad.distance_m  = 10.0;
    bad.speed_m_s   = 0.0;             // invalid
    EXPECT_FALSE(skill::agv_transport::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + origin/dest/distance/speed ───────────────
TEST(SkillAgvTransport, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::agv_transport::Input in;
    in.origin      = "WH-IN";
    in.destination = "LINE-3";
    in.distance_m  = 75.5;
    in.speed_m_s   = 0.8;

    auto out = skill::agv_transport::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("agv_transport"));
    EXPECT_EQ(out.signature.pattern["origin"].get<std::string>(),
              std::string("WH-IN"));
    EXPECT_EQ(out.signature.pattern["destination"].get<std::string>(),
              std::string("LINE-3"));
    EXPECT_NEAR(out.signature.pattern["distance_m"].get<double>(), 75.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["speed_m_s"].get<double>(),  0.8,  1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("agv"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillAgvTransport, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::agv_transport::Input in;
    in.origin      = "M-02";
    in.destination = "QC-01";
    in.distance_m  = 30.0;
    in.speed_m_s   = 1.5;

    auto out = skill::agv_transport::apply(*stock, in);
    auto cands = skill::agv_transport::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["destination"].get<std::string>(),
              std::string("QC-01"));
    EXPECT_NEAR(cands[0].recovered_params["distance_m"].get<double>(), 30.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::agv_transport::recognize(raw).empty());
}

// ── 5. est_cycle_time_s = 5 + distance / speed (specific assertion) ──────
TEST(SkillAgvTransport, CycleTimeMatchesDistanceOverSpeed)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::agv_transport::Input in;
    in.origin      = "M-01";
    in.destination = "A-01";
    in.distance_m  = 60.0;
    in.speed_m_s   = 1.0;

    auto out = skill::agv_transport::apply(*stock, in);
    EXPECT_NEAR(out.signature.tooling.est_cycle_time_s, 65.0, 1e-9);

    // Degenerate case: origin == destination → warning, still passes.
    skill::agv_transport::Input deg;
    deg.origin      = "DOCK";
    deg.destination = "DOCK";
    deg.distance_m  = 1.0;
    deg.speed_m_s   = 1.0;
    auto r = skill::agv_transport::validate(*stock, deg);
    EXPECT_TRUE(r.passed);
    bool warned = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-AGV-DEGENERATE" && f.severity == "warning") {
            warned = true; break;
        }
    }
    EXPECT_TRUE(warned);
}
