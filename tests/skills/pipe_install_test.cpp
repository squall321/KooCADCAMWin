// @lat: [[process/test-strategy#skill round-trip]]
//
// pipe_install skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range diameter, and metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pipe_install.hpp"

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

skill::pipe_install::Input goodInput()
{
    skill::pipe_install::Input in;
    in.pipe_dia_mm   = 25.0;
    in.pipe_material = "copper";
    in.length_m      = 10.0;
    in.joint_count   = 4;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPipeInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::pipe_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("pipe_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (oversize diameter) ────────────────────
TEST(SkillPipeInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pipe_dia_mm = 800.0;    // > 600 mm upper limit
    auto r = skill::pipe_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PIPE-DIA"));
    EXPECT_THROW(skill::pipe_install::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.length_m = 0.0;        // invalid length
    auto r2 = skill::pipe_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillPipeInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.pipe_dia_mm   = 50.0;
    in.pipe_material = "stainless";
    in.length_m      = 15.0;
    in.joint_count   = 6;

    auto out = skill::pipe_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pipe_install"));
    EXPECT_NEAR(out.signature.pattern["pipe_dia_mm"].get<double>(), 50.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["pipe_material"].get<std::string>(),
              std::string("stainless"));
    EXPECT_NEAR(out.signature.pattern["length_m"].get<double>(), 15.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["joint_count"].get<int>(), 6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPipeInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::pipe_install::apply(*stock, goodInput());

    auto cands = skill::pipe_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["pipe_dia_mm"].get<double>(),
                25.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::pipe_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "pipe_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: high joint density emits info finding ───────────────────
TEST(SkillPipeInstall, HighJointDensityEmitsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.length_m    = 5.0;
    in.joint_count = 20;       // 4 joints/m > 2 threshold

    auto r = skill::pipe_install::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "info-level finding must not block";
    EXPECT_TRUE(hasFinding(r, "DFM-PIPE-JOINTS"));

    // apply() still succeeds.
    EXPECT_NO_THROW(skill::pipe_install::apply(*stock, in));
}
