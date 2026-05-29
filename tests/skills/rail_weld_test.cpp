// @lat: [[process/test-strategy#skill round-trip]]
//
// rail_weld skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of bad weld method, metadata-only recognition, and
// flash-butt vs thermite tooling selection.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rail_weld.hpp"

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

skill::rail_weld::Input goodInput()
{
    skill::rail_weld::Input in;
    in.weld_method = "thermite";
    in.rail_grade  = "R260";
    in.joint_count = 4;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillRailWeld, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::rail_weld::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("rail_weld"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (bad weld method + empty grade) ────────
TEST(SkillRailWeld, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.weld_method = "laser";          // not in {thermite, flash_butt}
    auto r = skill::rail_weld::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-RW-METHOD"));
    EXPECT_THROW(skill::rail_weld::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.rail_grade  = "";
    in2.joint_count = 0;
    auto r2 = skill::rail_weld::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + method/grade/joint_count ─────────────────
TEST(SkillRailWeld, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.weld_method = "flash_butt";
    in.rail_grade  = "R350HT";
    in.joint_count = 12;
    auto out = skill::rail_weld::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rail_weld"));
    EXPECT_EQ(out.signature.pattern["weld_method"].get<std::string>(),
              std::string("flash_butt"));
    EXPECT_EQ(out.signature.pattern["rail_grade"].get<std::string>(),
              std::string("R350HT"));
    EXPECT_EQ(out.signature.pattern["joint_count"].get<int>(), 12);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRailWeld, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::rail_weld::apply(*stock, goodInput());

    auto cands = skill::rail_weld::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["weld_method"].get<std::string>(),
              std::string("thermite"));
    EXPECT_EQ(cands[0].recovered_params["joint_count"].get<int>(), 4);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::rail_weld::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "rail_weld cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: flash-butt tooling is the welder, thermite is crucible ──
TEST(SkillRailWeld, ToolingTypeReflectsWeldMethod)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto inT = goodInput();
    inT.weld_method = "thermite";
    inT.joint_count = 2;
    auto outT = skill::rail_weld::apply(*stock, inT);
    EXPECT_EQ(outT.signature.tooling.tool_type,
              std::string("thermite_crucible_kit"));

    auto inF = goodInput();
    inF.weld_method = "flash_butt";
    inF.joint_count = 2;
    auto outF = skill::rail_weld::apply(*stock, inF);
    EXPECT_EQ(outF.signature.tooling.tool_type,
              std::string("flash_butt_welder"));

    // Flash-butt should be markedly faster than thermite.
    EXPECT_LT(outF.signature.tooling.est_cycle_time_s,
              outT.signature.tooling.est_cycle_time_s);
}
