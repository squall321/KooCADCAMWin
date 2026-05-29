// @lat: [[process/test-strategy#skill round-trip]]
//
// friction_stir_weld_aero skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of off-window tool diameter and traverse speed,
// metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/friction_stir_weld_aero.hpp"

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

skill::friction_stir_weld_aero::Input goodInput()
{
    skill::friction_stir_weld_aero::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.tool_dia_mm           = 16.0;
    in.traverse_speed_mm_min = 400.0;
    in.seam_length_mm        = 300.0;
    in.alloy_pair            = "AA2024-AA7075";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillFrictionStirWeldAero, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(400.0, 100.0, 5.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::friction_stir_weld_aero::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("friction_stir_weld_aero"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (tool 30 mm > 25 max, speed off) ──────
TEST(SkillFrictionStirWeldAero, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(400.0, 100.0, 5.0);

    auto in = goodInput();
    in.tool_dia_mm = 30.0;        // > 25 mm max
    auto r = skill::friction_stir_weld_aero::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FSW-TOOL"));
    EXPECT_THROW(skill::friction_stir_weld_aero::apply(*stock, in),
                 skill::SkillError);

    auto in2 = goodInput();
    in2.traverse_speed_mm_min = 30.0;   // < 50
    auto r2 = skill::friction_stir_weld_aero::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-FSW-SPEED"));
}

// ─── 3. Signature records kind + tool_dia / speed / alloy_pair ──────────
TEST(SkillFrictionStirWeldAero, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(400.0, 100.0, 5.0);

    auto in = goodInput();
    in.tool_dia_mm           = 12.0;
    in.traverse_speed_mm_min = 250.0;
    in.alloy_pair            = "AA2195-AA2050";

    auto out = skill::friction_stir_weld_aero::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("friction_stir_weld_aero"));
    EXPECT_NEAR(out.signature.pattern["tool_dia_mm"].get<double>(), 12.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["traverse_speed_mm_min"].get<double>(),
                250.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["alloy_pair"].get<std::string>(),
              std::string("AA2195-AA2050"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("fsw_shoulder_pin"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFrictionStirWeldAero, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(400.0, 100.0, 5.0);
    auto out = skill::friction_stir_weld_aero::apply(*stock, goodInput());

    auto cands = skill::friction_stir_weld_aero::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["alloy_pair"].get<std::string>(),
              std::string("AA2024-AA7075"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::friction_stir_weld_aero::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "friction_stir_weld_aero cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: cycle time scales linearly with seam length ─────────────
TEST(SkillFrictionStirWeldAero, CycleTimeScalesWithSeamLength)
{
    auto stock = skill::createCuboidStock(2000.0, 100.0, 5.0);

    auto shortIn = goodInput();
    shortIn.seam_length_mm = 100.0;
    auto shortOut = skill::friction_stir_weld_aero::apply(*stock, shortIn);

    auto longIn = goodInput();
    longIn.seam_length_mm = 1000.0;
    auto longOut = skill::friction_stir_weld_aero::apply(*stock, longIn);

    // Longer seam ⇒ longer cycle time at same traverse_speed.
    EXPECT_GT(longOut.signature.tooling.est_cycle_time_s,
              shortOut.signature.tooling.est_cycle_time_s);

    // Approximate ratio: 1000/100 = 10× longer (since dominant term is
    // seam_length / traverse_speed × 60).
    const double ratio = longOut.signature.tooling.est_cycle_time_s
                       / shortOut.signature.tooling.est_cycle_time_s;
    EXPECT_GT(ratio, 5.0);
}
