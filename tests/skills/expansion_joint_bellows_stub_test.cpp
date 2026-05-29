// @lat: [[process/test-strategy#skill round-trip]]
//
// expansion_joint_bellows_stub — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/expansion_joint_bellows_stub.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

skill::expansion_joint_bellows_stub::Input goodInput()
{
    skill::expansion_joint_bellows_stub::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 80.0;
    in.center_y_mm           = 80.0;
    in.axis_dir              = gp_Dir(0, 0, 1);
    in.flange_outer_dia_mm   = 150.0;
    in.flange_thickness_mm   = 14.0;
    in.pipe_outer_dia_mm     = 80.0;
    in.pipe_inner_dia_mm     = 70.0;
    in.pipe_length_mm        = 60.0;
    in.corrugation_count     = 3;
    in.corrugation_pitch_mm  = 16.0;
    in.corrugation_outer_dia_mm = 100.0;
    return in;
}

}  // namespace

// ─── 1. Apply produces REAL volume delta matching expected geometry ───────
TEST(SkillExpansionJointBellowsStub, ApplyProducesExpectedDelta)
{
    // Thin stock pad; flange & pipe fuse on top.
    auto stock = skill::createCuboidStock(160.0, 160.0, 4.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::expansion_joint_bellows_stub::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected ADDED ≈ flange + pipe + torus stack.
    // Expected REMOVED ≈ central bore through (flange + pipe).
    const double Rflange = in.flange_outer_dia_mm / 2.0;
    const double Rpipe   = in.pipe_outer_dia_mm   / 2.0;
    const double Rbore   = in.pipe_inner_dia_mm   / 2.0;
    const double minorR  = (in.corrugation_outer_dia_mm - in.pipe_outer_dia_mm) / 2.0;
    const double majorR  = in.pipe_outer_dia_mm / 2.0 + minorR;
    const double vFlange = M_PI * Rflange * Rflange * in.flange_thickness_mm;
    const double vPipe   = M_PI * Rpipe   * Rpipe   * in.pipe_length_mm;
    const double vTorus  = 2.0 * M_PI * M_PI * majorR * minorR * minorR *
                           static_cast<double>(in.corrugation_count);
    const double vBore   = M_PI * Rbore * Rbore *
                           (in.flange_thickness_mm + in.pipe_length_mm);
    const double expectedDelta = (vFlange + vPipe + vTorus) - vBore;

    const double actualDelta = volumeOf(out.workpiece->shape()) - volBefore;
    EXPECT_GT(actualDelta, 0.0);
    // Wide tolerance — toroid/pipe overlap subtracted by fuse.
    EXPECT_NEAR(actualDelta, expectedDelta, std::abs(expectedDelta) * 0.20)
        << "actual=" << actualDelta << " expected≈" << expectedDelta;

    // Face count grew (corrugations alone add ≥ N tori faces).
    EXPECT_GT(out.workpiece->faceCount(), facesBefore + in.corrugation_count);
}

// ─── 2. Validate rejects too few convolutions + apply throws ──────────────
TEST(SkillExpansionJointBellowsStub, ValidateRejectsTooFewConvolutions)
{
    auto stock = skill::createCuboidStock(160.0, 160.0, 4.0);
    auto in    = goodInput();
    in.corrugation_count = 1;   // < min 2

    auto r = skill::expansion_joint_bellows_stub::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-EJMA-10"));
    EXPECT_THROW(skill::expansion_joint_bellows_stub::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature flags is_compound=true & params round-trip ──────────────
TEST(SkillExpansionJointBellowsStub, SignatureCompoundAndParamsRoundTrip)
{
    auto stock = skill::createCuboidStock(160.0, 160.0, 4.0);
    auto in    = goodInput();
    auto out   = skill::expansion_joint_bellows_stub::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("expansion_joint_bellows_stub"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    const int subCount = out.signature.pattern.value("subfeature_count", 0);
    EXPECT_GE(subCount, 2);
    EXPECT_EQ(subCount, 3 + in.corrugation_count);
    EXPECT_EQ(out.signature.params.value("corrugation_count", -1),
              in.corrugation_count);
}

// ─── 4. Recognize returns a candidate with confidence > 0.5 ───────────────
TEST(SkillExpansionJointBellowsStub, RecognizeFindsCompound)
{
    auto stock = skill::createCuboidStock(160.0, 160.0, 4.0);
    auto in    = goodInput();
    auto out   = skill::expansion_joint_bellows_stub::apply(*stock, in);

    auto cands = skill::expansion_joint_bellows_stub::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence > 0.5) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

// ─── 5. Feature-specific: corrugation count recovered ─────────────────────
TEST(SkillExpansionJointBellowsStub, RecoveredCorrugationCount)
{
    auto stock = skill::createCuboidStock(160.0, 160.0, 4.0);
    auto in    = goodInput();
    in.corrugation_count = 4;
    auto out   = skill::expansion_joint_bellows_stub::apply(*stock, in);

    auto cands = skill::expansion_joint_bellows_stub::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        const int n = c.recovered_params.value("corrugation_count", 0);
        if (n == in.corrugation_count) { found = true; break; }
    }
    EXPECT_TRUE(found);
}
