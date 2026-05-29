// @lat: [[process/test-strategy#skill round-trip]]
//
// crane_lift — 5-case sweep covering identity-geometry synthesis,
// capacity / height DFM gates, metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/crane_lift.hpp"

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
TEST(SkillCraneLift, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::crane_lift::Input in;
    in.lifting_capacity_kg = 1500.0;
    in.lift_height_m       = 4.0;
    in.lift_class          = "standard";

    auto out = skill::crane_lift::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("crane_lift"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero / negative capacity & height rejected ───────────────────
TEST(SkillCraneLift, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::crane_lift::Input bad;
    bad.lifting_capacity_kg = 0.0;       // invalid
    bad.lift_height_m       = 5.0;
    auto r = skill::crane_lift::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::crane_lift::apply(*stock, bad), skill::SkillError);

    bad.lifting_capacity_kg = -100.0;
    EXPECT_FALSE(skill::crane_lift::validate(*stock, bad).passed);

    bad.lifting_capacity_kg = 1000.0;
    bad.lift_height_m       = 0.0;       // invalid
    EXPECT_FALSE(skill::crane_lift::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + capacity/height/class ────────────────────
TEST(SkillCraneLift, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::crane_lift::Input in;
    in.lifting_capacity_kg = 10000.0;
    in.lift_height_m       = 8.5;
    in.lift_class          = "heavy";

    auto out = skill::crane_lift::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("crane_lift"));
    EXPECT_NEAR(out.signature.pattern["lifting_capacity_kg"].get<double>(),
                10000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["lift_height_m"].get<double>(),
                8.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["lift_class"].get<std::string>(),
              std::string("heavy"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("overhead_crane"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillCraneLift, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::crane_lift::Input in;
    in.lifting_capacity_kg = 3000.0;
    in.lift_height_m       = 6.0;
    in.lift_class          = "standard";

    auto out = skill::crane_lift::apply(*stock, in);
    auto cands = skill::crane_lift::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["lifting_capacity_kg"].get<double>(),
                3000.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["lift_class"].get<std::string>(),
              std::string("standard"));

    // Same shape without metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::crane_lift::recognize(raw).empty());
}

// ── 5. Excessive capacity yields info finding, still passes ──────────────
TEST(SkillCraneLift, ExcessCapacityIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::crane_lift::Input in;
    in.lifting_capacity_kg = 60000.0;     // above typical 50 t
    in.lift_height_m       = 6.0;
    in.lift_class          = "ultra";

    auto r = skill::crane_lift::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "excess capacity should NOT block (info-only)";
    bool informed = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-CRANE-CAPACITY" && f.severity == "info") {
            informed = true; break;
        }
    }
    EXPECT_TRUE(informed);
    EXPECT_NO_THROW(skill::crane_lift::apply(*stock, in));
}
