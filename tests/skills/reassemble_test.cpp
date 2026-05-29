// @lat: [[process/test-strategy#skill round-trip]]
//
// reassemble skill — 5-case sweep:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific: torque_specs_count > part_count emits info but does not block

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/reassemble.hpp"

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

}  // namespace

// ─── 1. Apply: geometry COMPLETELY unchanged ─────────────────────────────
TEST(SkillReassemble, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::reassemble::Input in;
    in.reassembly_order   = { "shaft", "bearing", "cover" };
    in.torque_specs_count = 4;

    auto out = skill::reassemble::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("reassemble"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillReassemble, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::reassemble::Input emptyOrder;
    emptyOrder.torque_specs_count = 0;
    EXPECT_FALSE(skill::reassemble::validate(*stock, emptyOrder).passed);
    EXPECT_THROW(skill::reassemble::apply(*stock, emptyOrder),
                 skill::SkillError);

    skill::reassemble::Input negTorque;
    negTorque.reassembly_order   = { "A", "B" };
    negTorque.torque_specs_count = -1;
    EXPECT_FALSE(skill::reassemble::validate(*stock, negTorque).passed);
    EXPECT_THROW(skill::reassemble::apply(*stock, negTorque),
                 skill::SkillError);
}

// ─── 3. Signature records kind + params ──────────────────────────────────
TEST(SkillReassemble, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::reassemble::Input in;
    in.reassembly_order   = { "housing", "rotor", "stator", "endcap" };
    in.torque_specs_count = 8;

    auto out = skill::reassemble::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("reassemble"));
    ASSERT_TRUE(out.signature.pattern["reassembly_order"].is_array());
    EXPECT_EQ(out.signature.pattern["reassembly_order"].size(), 4u);
    EXPECT_EQ(out.signature.pattern["reassembly_order"][0].get<std::string>(),
              std::string("housing"));
    EXPECT_EQ(out.signature.pattern["reassembly_order"][3].get<std::string>(),
              std::string("endcap"));
    EXPECT_EQ(out.signature.pattern["part_count"].get<int>(), 4);
    EXPECT_EQ(out.signature.pattern["torque_specs_count"].get<int>(), 8);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.params["torque_specs_count"].get<int>(), 8);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillReassemble, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::reassemble::Input in;
    in.reassembly_order   = { "p1", "p2" };
    in.torque_specs_count = 2;
    auto out = skill::reassemble::apply(*stock, in);

    auto cands = skill::reassemble::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    ASSERT_TRUE(cands[0].recovered_params["reassembly_order"].is_array());
    EXPECT_EQ(cands[0].recovered_params["reassembly_order"].size(), 2u);
    EXPECT_EQ(cands[0].recovered_params["torque_specs_count"].get<int>(), 2);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::reassemble::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: torque_specs_count > part_count emits info, not block ──
TEST(SkillReassemble, ExcessTorqueSpecsAreInfoOnly)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::reassemble::Input in;
    in.reassembly_order   = { "A", "B" };
    in.torque_specs_count = 5;           // > 2 parts

    auto r = skill::reassemble::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "torque overcount is info-only, should not block";
    EXPECT_TRUE(hasFinding(r, "DFM-RA-TORQUE"));
    EXPECT_NO_THROW(skill::reassemble::apply(*stock, in));
}
