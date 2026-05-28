// @lat: [[process/test-strategy#skill round-trip]]
//
// resin_transfer_mold skill — metadata-only RTM process recording.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/resin_transfer_mold.hpp"

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

// ─── 1. Apply handles input + geometry unchanged ──────────────────────────
TEST(SkillResinTransferMold, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(70.0, 70.0, 3.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::resin_transfer_mold::Input in;
    in.injection_pressure_kpa = 700.0;
    in.gate_count             = 2;
    in.vent_count             = 2;

    auto out = skill::resin_transfer_mold::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("resin_transfer_mold"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (zero gates) ───────────────────────────
TEST(SkillResinTransferMold, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    skill::resin_transfer_mold::Input in;
    in.injection_pressure_kpa = 700.0;
    in.gate_count             = 0;       // < 1 → DFM-RTM-GATES error
    in.vent_count             = 1;

    auto r = skill::resin_transfer_mold::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RTM-GATES") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::resin_transfer_mold::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillResinTransferMold, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    skill::resin_transfer_mold::Input in;
    in.injection_pressure_kpa = 1200.0;
    in.gate_count             = 3;
    in.vent_count             = 4;

    auto out = skill::resin_transfer_mold::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("resin_transfer_mold"));
    EXPECT_NEAR(out.signature.pattern["injection_pressure_kpa"].get<double>(),
                1200.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["gate_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["vent_count"].get<int>(), 4);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("rtm_mold"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillResinTransferMold, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    skill::resin_transfer_mold::Input in;
    in.injection_pressure_kpa = 800.0;
    in.gate_count             = 1;
    in.vent_count             = 2;

    auto out = skill::resin_transfer_mold::apply(*stock, in);
    auto cands = skill::resin_transfer_mold::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["gate_count"].get<int>(), 1);
    EXPECT_EQ(cands[0].recovered_params["vent_count"].get<int>(), 2);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::resin_transfer_mold::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: gate/vent ratio > 5:1 emits info finding (not error) ────
TEST(SkillResinTransferMold, HighGateToVentRatioInfoFinding)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 3.0);

    skill::resin_transfer_mold::Input in;
    in.injection_pressure_kpa = 700.0;
    in.gate_count             = 12;
    in.vent_count             = 2;        // 12/2 = 6 > 5

    auto r = skill::resin_transfer_mold::validate(*stock, in);
    EXPECT_TRUE(r.passed);            // info-only, not error
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-RTM-RATIO" && f.severity == "info") {
            found = true; break;
        }
    EXPECT_TRUE(found);
}
