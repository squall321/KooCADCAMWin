// @lat: [[process/test-strategy#skill round-trip]]
//
// ball_inflate skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of burst-risk pressure, metadata-only recognition, and a
// skill-specific assertion that in-spec ranges are recognised per ball type.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ball_inflate.hpp"

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

skill::ball_inflate::Input goodInput()
{
    skill::ball_inflate::Input in;
    in.target_pressure_psi = 9.0;     // mid-range for soccer (8.5-15.6)
    in.ball_type           = "soccer";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBallInflate, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::ball_inflate::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("ball_inflate"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillBallInflate, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // Burst-risk pressure (> 30 psi).
    auto in = goodInput();
    in.target_pressure_psi = 45.0;
    auto r = skill::ball_inflate::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BALL-PRESSURE"));
    EXPECT_THROW(skill::ball_inflate::apply(*stock, in), skill::SkillError);

    // Zero pressure (input error).
    auto in2 = goodInput();
    in2.target_pressure_psi = 0.0;
    auto r2 = skill::ball_inflate::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillBallInflate, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.target_pressure_psi = 13.0;
    in.ball_type           = "football";

    auto out = skill::ball_inflate::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ball_inflate"));
    EXPECT_NEAR(out.signature.pattern["target_pressure_psi"].get<double>(),
                13.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["ball_type"].get<std::string>(),
              std::string("football"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBallInflate, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::ball_inflate::apply(*stock, goodInput());

    auto cands = skill::ball_inflate::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["target_pressure_psi"].get<double>(),
                9.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["ball_type"].get<std::string>(),
              std::string("soccer"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::ball_inflate::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "ball_inflate cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: in_spec_range flips with type-specific spec range ──────
TEST(SkillBallInflate, InSpecRangeFlipsPerType)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // Soccer ball at 9.0 psi → within [8.5, 15.6] → in_spec
    auto inA = goodInput();
    inA.ball_type = "soccer";
    inA.target_pressure_psi = 9.0;
    auto outA = skill::ball_inflate::apply(*stock, inA);
    EXPECT_TRUE(outA.signature.pattern["in_spec_range"].get<bool>());

    // Basketball at 9.0 psi → outside [7.5, 8.5] → out of spec (info only)
    auto inB = goodInput();
    inB.ball_type = "basketball";
    inB.target_pressure_psi = 9.0;
    auto outB = skill::ball_inflate::apply(*stock, inB);
    EXPECT_FALSE(outB.signature.pattern["in_spec_range"].get<bool>());

    // Spec range bounds should be present in pattern for known types.
    EXPECT_NEAR(outA.signature.pattern["spec_lo_psi"].get<double>(),  8.5, 1e-9);
    EXPECT_NEAR(outA.signature.pattern["spec_hi_psi"].get<double>(), 15.6, 1e-9);
}
