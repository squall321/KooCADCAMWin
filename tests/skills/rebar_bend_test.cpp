// @lat: [[process/test-strategy#skill round-trip]]
//
// rebar_bend skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of sub-3D bend radius, metadata-only recognition, and
// the specific assertion that bend_ratio is recorded in the signature.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rebar_bend.hpp"

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

skill::rebar_bend::Input goodInput()
{
    skill::rebar_bend::Input in;
    in.dia_mm         = 16.0;
    in.bend_angle_deg = 90.0;
    in.bend_radius_mm = 64.0;          // 4 × dia, comfortably above 3 × min
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillRebarBend, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::rebar_bend::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("rebar_bend"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (bend_radius < 3 × dia) ────────────────
TEST(SkillRebarBend, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.bend_radius_mm = 30.0;          // < 3 × 16 = 48 mm minimum
    auto r = skill::rebar_bend::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-REBAR-RADIUS"));
    EXPECT_THROW(skill::rebar_bend::apply(*stock, in), skill::SkillError);

    // Angle out of range
    auto in2 = goodInput();
    in2.bend_angle_deg = 200.0;        // > 180
    auto r2 = skill::rebar_bend::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + dia/angle/radius ────────────────────────
TEST(SkillRebarBend, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.dia_mm         = 19.0;
    in.bend_angle_deg = 135.0;
    in.bend_radius_mm = 76.0;          // 4 × dia

    auto out = skill::rebar_bend::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rebar_bend"));
    EXPECT_NEAR(out.signature.pattern["dia_mm"].get<double>(),         19.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["bend_angle_deg"].get<double>(), 135.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["bend_radius_mm"].get<double>(), 76.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRebarBend, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::rebar_bend::apply(*stock, goodInput());

    auto cands = skill::rebar_bend::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["dia_mm"].get<double>(), 16.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::rebar_bend::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "rebar_bend cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: bend_ratio is exactly bend_radius / dia ─────────────────
TEST(SkillRebarBend, BendRatioRecordedAsRadiusOverDia)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.dia_mm         = 22.0;
    in.bend_radius_mm = 110.0;         // ratio = 5.0
    in.bend_angle_deg = 90.0;
    auto out = skill::rebar_bend::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern["bend_ratio"].get<double>(), 5.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("rebar_bender"));
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 22.0, 1e-9);
}
