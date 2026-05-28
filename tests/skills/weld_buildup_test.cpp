// @lat: [[process/test-strategy#skill round-trip]]
//
// weld_buildup — additive cladding: deposits a thin slab on a planar face.
//
// Cases (6):
//   1. apply fuses a slab onto the top face and ADDS volume.
//   2. DFM-BUILDUP-THICK rejects thickness < 0.5 mm.
//   3. DFM-BUILDUP-THICK rejects thickness > 10 mm.
//   4. recognize() replays the buildup metadata.
//   5. Geometric recognize finds a thin lid on a feature-less workpiece.
//   6. Multiple buildups can be chained onto different faces.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/weld_buildup.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. Apply: slab is fused, volume increases ─────────────────────────────
TEST(SkillWeldBuildup, ApplyAddsSlabVolume)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    skill::weld_buildup::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.buildup_thickness_mm  = 2.0;
    in.material              = "stainless_steel";

    auto out = skill::weld_buildup::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double newVol = volumeOf(out.workpiece->shape());
    const double added  = newVol - stockVol;
    // The slab footprint is the bbox of the top face = 50 × 30 = 1500 mm².
    // Added volume ≈ 1500 × 2.0 = 3000 mm³ (minus tiny sink overlap).
    const double approxAdded = 50.0 * 30.0 * 2.0;
    EXPECT_GT(added, 0.0);
    EXPECT_NEAR(added, approxAdded, approxAdded * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("weld_buildup"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_TRUE(out.signature.pattern["additive"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["thickness_mm"].get<double>(), 2.0, 1e-9);
}

// ─── 2. DFM rejects sub-0.5 mm buildup ─────────────────────────────────────
TEST(SkillWeldBuildup, ValidateRejectsTooThin)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 10.0);

    skill::weld_buildup::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.buildup_thickness_mm  = 0.2;        // < 0.5
    in.material              = "stainless_steel";

    auto r = skill::weld_buildup::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BUILDUP-THICK" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::weld_buildup::apply(*stock, in), skill::SkillError);
}

// ─── 3. DFM rejects >10 mm buildup ─────────────────────────────────────────
TEST(SkillWeldBuildup, ValidateRejectsTooThick)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 10.0);

    skill::weld_buildup::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.buildup_thickness_mm  = 12.0;       // > 10
    in.material              = "stainless_steel";

    auto r = skill::weld_buildup::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-BUILDUP-THICK" && f.severity == "error") {
            found = true;
            break;
        }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::weld_buildup::apply(*stock, in), skill::SkillError);
}

// ─── 4. Recognize replays history ──────────────────────────────────────────
TEST(SkillWeldBuildup, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 10.0);

    skill::weld_buildup::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.buildup_thickness_mm  = 1.5;
    in.material              = "inconel_625";

    auto out   = skill::weld_buildup::apply(*stock, in);
    auto cands = skill::weld_buildup::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_NEAR(cands[0].recovered_params["buildup_thickness_mm"].get<double>(),
                1.5, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(),
              std::string("inconel_625"));
}

// ─── 5. Geometric recognize on a thin stock detects parallel-face pair ────
//
// Geometric recognition is inherently ambiguous: a slab fused flush onto a
// cuboid's top face merges into a homogeneous block.  The heuristic
// recognises buildups only when the resulting shape still exposes a pair
// of parallel planar faces separated by < 10 mm.  We construct that by
// using a thin workpiece where the buildup leaves the cuboid "tall but
// shallow", so the front/back pair (now sep = the original short dim)
// stays within range.
TEST(SkillWeldBuildup, RecognizeGeometricReturnsSomething)
{
    // 50 × 6 × 10 stock → after 2 mm buildup on top, fused shape is
    // 50 × 6 × 12.  Front-back pair (normal ±Y) has sep = 6 mm ≤ 10 mm,
    // so the heuristic should pick it up.
    auto stock = skill::createCuboidStock(50.0, 6.0, 10.0);

    skill::weld_buildup::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.buildup_thickness_mm  = 2.0;
    in.material              = "stainless_steel";

    auto out = skill::weld_buildup::apply(*stock, in);

    // Strip metadata: build a feature-less workpiece around the fused shape.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands = skill::weld_buildup::recognize(raw);

    // The heuristic may report multiple candidates (one per parent face);
    // we only verify it picks up at least one and reports a thickness in
    // the valid range.
    ASSERT_FALSE(cands.empty())
        << "geometric recognize should find at least one buildup candidate";
    EXPECT_LT(cands[0].confidence, 0.95);   // lower than metadata replay
    EXPECT_GT(cands[0].confidence, 0.0);
    const double t = cands[0].recovered_params["buildup_thickness_mm"].get<double>();
    EXPECT_GT(t, 0.0);
    EXPECT_LE(t, 10.0);
}

// ─── 6. Multiple buildups on the same workpiece accumulate ────────────────
TEST(SkillWeldBuildup, MultipleBuildupsChain)
{
    auto stock = skill::createCuboidStock(50.0, 30.0, 10.0);

    skill::weld_buildup::Input top;
    top.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    top.buildup_thickness_mm  = 1.0;
    top.material              = "stainless_steel";
    auto w1 = skill::weld_buildup::apply(*stock, top);

    skill::weld_buildup::Input bot;
    bot.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, -1), 5.0, "largest" };
    bot.buildup_thickness_mm  = 1.0;
    bot.material              = "stainless_steel";
    auto w2 = skill::weld_buildup::apply(*w1.workpiece, bot);

    ASSERT_FALSE(w2.workpiece->shape().IsNull());
    EXPECT_EQ(w2.workpiece->features().size(), 2u);
    EXPECT_GT(volumeOf(w2.workpiece->shape()),
              volumeOf(w1.workpiece->shape()));
}
