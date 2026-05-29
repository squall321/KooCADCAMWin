// @lat: [[process/test-strategy#skill round-trip]]
//
// weld_rebar skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of unknown weld type, metadata-only recognition, and the
// specific assertion that tack vs full welds select different torch tools
// and cycle-time scaling.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/weld_rebar.hpp"

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

skill::weld_rebar::Input goodInput()
{
    skill::weld_rebar::Input in;
    in.weld_type   = "lap";
    in.joint_count = 12;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillWeldRebar, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::weld_rebar::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("weld_rebar"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (unknown weld_type) ────────────────────
TEST(SkillWeldRebar, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.weld_type = "thermite";                  // not in {tack,full,lap,butt}
    auto r = skill::weld_rebar::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-WELD-TYPE"));
    EXPECT_THROW(skill::weld_rebar::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.joint_count = 0;                        // < 1
    auto r2 = skill::weld_rebar::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + weld_type + joint_count ─────────────────
TEST(SkillWeldRebar, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.weld_type   = "butt";
    in.joint_count = 7;
    auto out = skill::weld_rebar::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("weld_rebar"));
    EXPECT_EQ(out.signature.pattern["weld_type"].get<std::string>(),
              std::string("butt"));
    EXPECT_EQ(out.signature.pattern["joint_count"].get<int>(), 7);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWeldRebar, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::weld_rebar::apply(*stock, goodInput());

    auto cands = skill::weld_rebar::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["weld_type"].get<std::string>(),
              std::string("lap"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::weld_rebar::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "weld_rebar cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: tack selects mig_torch; tack is faster per joint ────────
TEST(SkillWeldRebar, TackVsFullDifferTorchAndSpeed)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto tackIn = goodInput();
    tackIn.weld_type   = "tack";
    tackIn.joint_count = 10;
    auto tackOut = skill::weld_rebar::apply(*stock, tackIn);
    EXPECT_EQ(tackOut.signature.tooling.tool_type, std::string("mig_torch"));

    auto fullIn = goodInput();
    fullIn.weld_type   = "full";
    fullIn.joint_count = 10;
    auto fullOut = skill::weld_rebar::apply(*stock, fullIn);
    EXPECT_EQ(fullOut.signature.tooling.tool_type, std::string("smaw_electrode"));

    // Same joint count → tack must be faster than full.
    EXPECT_LT(tackOut.signature.tooling.est_cycle_time_s,
              fullOut.signature.tooling.est_cycle_time_s);
}
