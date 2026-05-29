// @lat: [[process/test-strategy#skill round-trip]]
//
// bga_reball skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range ball diameter, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bga_reball.hpp"

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

skill::bga_reball::Input goodInput()
{
    skill::bga_reball::Input in;
    in.ball_dia_mm = 0.50;
    in.ball_count  = 256;
    in.ball_alloy  = "SAC305";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBgaReball, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::bga_reball::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("bga_reball"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillBgaReball, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ball_dia_mm = 0.05;            // < 0.20 mm
    auto r = skill::bga_reball::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BALL-DIA"));
    EXPECT_THROW(skill::bga_reball::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.ball_count = 5000;            // > 2500
    auto r2 = skill::bga_reball::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BALL-COUNT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillBgaReball, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.ball_dia_mm = 0.76;
    in.ball_count  = 484;
    in.ball_alloy  = "Sn63Pb37";

    auto out = skill::bga_reball::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("bga_reball"));
    EXPECT_NEAR(out.signature.pattern["ball_dia_mm"].get<double>(), 0.76, 1e-9);
    EXPECT_EQ(out.signature.pattern["ball_count"].get<int>(), 484);
    EXPECT_EQ(out.signature.pattern["ball_alloy"].get<std::string>(),
              std::string("Sn63Pb37"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBgaReball, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::bga_reball::apply(*stock, goodInput());

    auto cands = skill::bga_reball::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["ball_dia_mm"].get<double>(),
                0.50, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::bga_reball::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "bga_reball cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: Sn63Pb37 has lower reflow peak than SAC305 ──────────────
TEST(SkillBgaReball, LeadedAlloyHasLowerReflowPeak)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto sacIn = goodInput();
    sacIn.ball_alloy = "SAC305";
    auto sacOut = skill::bga_reball::apply(*stock, sacIn);

    auto pbIn = goodInput();
    pbIn.ball_alloy = "Sn63Pb37";
    auto pbOut = skill::bga_reball::apply(*stock, pbIn);

    EXPECT_LT(pbOut.signature.pattern["reflow_peak_c"].get<double>(),
              sacOut.signature.pattern["reflow_peak_c"].get<double>());
    EXPECT_EQ(sacOut.signature.tooling.tool_type,
              std::string("reball_reflow_oven"));
    // Stencil aperture count tracks ball_count.
    EXPECT_EQ(sacOut.signature.pattern["stencil_apertures"].get<int>(), 256);
}
