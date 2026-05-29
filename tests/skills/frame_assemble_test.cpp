// @lat: [[process/test-strategy#skill round-trip]]
//
// frame_assemble skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of unknown frame_type, metadata-only recognition, and the
// specific assertion that steel and wood frames select different fastener
// tools (torque_wrench vs framing_nailer) and that wood is faster.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/frame_assemble.hpp"

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

skill::frame_assemble::Input goodInput()
{
    skill::frame_assemble::Input in;
    in.frame_type   = "steel";
    in.member_count = 10;
    in.joint_count  = 12;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillFrameAssemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::frame_assemble::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("frame_assemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (unknown frame_type) ───────────────────
TEST(SkillFrameAssemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.frame_type = "concrete";                // not in {steel, wood}
    auto r = skill::frame_assemble::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FRAME-TYPE"));
    EXPECT_THROW(skill::frame_assemble::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.member_count = 1;                      // < 2
    auto r2 = skill::frame_assemble::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + frame_type + counts ─────────────────────
TEST(SkillFrameAssemble, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.frame_type   = "wood";
    in.member_count = 24;
    in.joint_count  = 48;                      // joints_per_member = 2.0
    auto out = skill::frame_assemble::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("frame_assemble"));
    EXPECT_EQ(out.signature.pattern["frame_type"].get<std::string>(),
              std::string("wood"));
    EXPECT_EQ(out.signature.pattern["member_count"].get<int>(), 24);
    EXPECT_EQ(out.signature.pattern["joint_count"].get<int>(),  48);
    EXPECT_NEAR(out.signature.pattern["joints_per_member"].get<double>(),
                2.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFrameAssemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::frame_assemble::apply(*stock, goodInput());

    auto cands = skill::frame_assemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["frame_type"].get<std::string>(),
              std::string("steel"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::frame_assemble::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "frame_assemble cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: steel uses torque_wrench; wood uses framing_nailer; ─────
//      same joint_count → wood is faster than steel
TEST(SkillFrameAssemble, FrameTypeSelectsToolAndSpeed)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto steelIn = goodInput();
    steelIn.frame_type   = "steel";
    steelIn.member_count = 10;
    steelIn.joint_count  = 10;
    auto steelOut = skill::frame_assemble::apply(*stock, steelIn);
    EXPECT_EQ(steelOut.signature.tooling.tool_type,
              std::string("torque_wrench"));

    auto woodIn = goodInput();
    woodIn.frame_type   = "wood";
    woodIn.member_count = 10;
    woodIn.joint_count  = 10;
    auto woodOut = skill::frame_assemble::apply(*stock, woodIn);
    EXPECT_EQ(woodOut.signature.tooling.tool_type,
              std::string("framing_nailer"));

    // Same joint count → wood (nailer) must be much faster than steel.
    EXPECT_LT(woodOut.signature.tooling.est_cycle_time_s,
              steelOut.signature.tooling.est_cycle_time_s);
}
