// @lat: [[process/test-strategy#skill round-trip]]
//
// hoist_op — 5-case sweep covering identity-geometry synthesis,
// hoist type / capacity / height DFM gates, metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hoist_op.hpp"

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

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillHoistOp, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::hoist_op::Input in;
    in.hoist_type        = "chain";
    in.hoist_capacity_kg = 500.0;
    in.lift_height_m     = 2.5;

    auto out = skill::hoist_op::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("hoist_op"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero / negative capacity & height rejected ───────────────────
TEST(SkillHoistOp, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::hoist_op::Input bad;
    bad.hoist_type        = "chain";
    bad.hoist_capacity_kg = 0.0;           // invalid
    bad.lift_height_m     = 3.0;
    auto r = skill::hoist_op::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::hoist_op::apply(*stock, bad), skill::SkillError);

    bad.hoist_capacity_kg = -200.0;
    EXPECT_FALSE(skill::hoist_op::validate(*stock, bad).passed);

    bad.hoist_capacity_kg = 500.0;
    bad.lift_height_m     = 0.0;            // invalid
    EXPECT_FALSE(skill::hoist_op::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + hoist_type/capacity/height ───────────────
TEST(SkillHoistOp, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::hoist_op::Input in;
    in.hoist_type        = "wire_rope";
    in.hoist_capacity_kg = 2500.0;
    in.lift_height_m     = 6.0;

    auto out = skill::hoist_op::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("hoist_op"));
    EXPECT_EQ(out.signature.pattern["hoist_type"].get<std::string>(),
              std::string("wire_rope"));
    EXPECT_NEAR(out.signature.pattern["hoist_capacity_kg"].get<double>(),
                2500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["lift_height_m"].get<double>(),
                6.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("hoist"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillHoistOp, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::hoist_op::Input in;
    in.hoist_type        = "lever";
    in.hoist_capacity_kg = 750.0;
    in.lift_height_m     = 1.5;

    auto out = skill::hoist_op::apply(*stock, in);
    auto cands = skill::hoist_op::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["hoist_type"].get<std::string>(),
              std::string("lever"));
    EXPECT_NEAR(cands[0].recovered_params["hoist_capacity_kg"].get<double>(),
                750.0, 1e-9);

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::hoist_op::recognize(raw).empty());
}

// ── 5. Unknown hoist_type → info finding, not block ──────────────────────
TEST(SkillHoistOp, UnknownHoistTypeEmitsInfoAndProceeds)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::hoist_op::Input in;
    in.hoist_type        = "electric_air";       // not in known set
    in.hoist_capacity_kg = 500.0;
    in.lift_height_m     = 3.0;

    auto r = skill::hoist_op::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "unrecognised hoist_type should NOT block (info-only)";
    bool informed = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-HOIST-TYPE" && f.severity == "info") {
            informed = true; break;
        }
    }
    EXPECT_TRUE(informed);
    EXPECT_NO_THROW(skill::hoist_op::apply(*stock, in));
}
