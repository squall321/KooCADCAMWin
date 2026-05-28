// @lat: [[process/test-strategy#skill round-trip]]
//
// squaring skill — 5-case sweep.  Subtractive bbox-fit op:
//   1. ApplyHandlesInput          — geometry shrinks, signature appended
//   2. ValidateRejectsBadInput    — negative margin → error → throw
//   3. SignatureRecordsKind+Keys  — pattern.kind and material-per-face exist
//   4. RecognizeMetadataReplay    — feature replay returns conf 1.0
//   5. Specific assertion         — final volume ≈ inset bbox volume

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/squaring.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
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

// ─── 1. Apply: trimming margin removes volume + appends signature ─────────
TEST(SkillSquaring, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 30.0);
    const double volBefore = volumeOf(stock->shape());

    skill::squaring::Input in;
    in.target_envelope_margin_mm = 1.0;

    auto out = skill::squaring::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    EXPECT_LT(volAfter, volBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("squaring"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate: negative margin rejected ────────────────────────────────
TEST(SkillSquaring, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 30.0);

    skill::squaring::Input in;
    in.target_envelope_margin_mm = -1.0;

    auto r = skill::squaring::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::squaring::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + per-face material removed ────────────────
TEST(SkillSquaring, SignatureRecordsKindAndKeys)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 30.0);

    skill::squaring::Input in;
    in.target_envelope_margin_mm = 0.5;

    auto out = skill::squaring::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("squaring"));
    ASSERT_TRUE(out.signature.pattern.contains("material_removed_per_face"));
    const auto& mpf = out.signature.pattern["material_removed_per_face"];
    for (const char* k : { "+x", "-x", "+y", "-y", "+z", "-z" }) {
        ASSERT_TRUE(mpf.contains(k));
        EXPECT_GE(mpf[k].get<double>(), 0.0);
    }
    ASSERT_TRUE(out.signature.pattern.contains("envelope_size_mm"));
    EXPECT_GT(out.signature.tooling.stock_removed_mm3, 0.0);
}

// ─── 4. Recognize metadata replay (after apply, returns conf 1.0) ─────────
TEST(SkillSquaring, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 30.0);

    skill::squaring::Input in;
    in.target_envelope_margin_mm = 0.5;
    auto out = skill::squaring::apply(*stock, in);

    auto cands = skill::squaring::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    bool foundReplay = false;
    for (const auto& c : cands) {
        if (std::abs(c.confidence - 1.0) < 1e-6) {
            foundReplay = true;
            EXPECT_EQ(c.recovered_params["target_envelope_margin_mm"].get<double>(),
                      0.5);
            break;
        }
    }
    EXPECT_TRUE(foundReplay);
}

// ─── 5. Specific assertion: final bbox == nominal envelope ────────────────
TEST(SkillSquaring, FinalBboxMatchesInsetEnvelope)
{
    auto stock = skill::createCuboidStock(50.0, 40.0, 30.0);

    skill::squaring::Input in;
    in.target_envelope_margin_mm = 1.0;
    auto out = skill::squaring::apply(*stock, in);

    Bnd_Box b;
    BRepBndLib::AddOptimal(out.workpiece->shape(), b);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    b.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    // Cuboid stock is already axis-aligned; intersecting with the inset
    // bbox should yield a (50−2) × (40−2) × (30−2) brick.
    EXPECT_NEAR(xMax - xMin, 48.0, 1e-3);
    EXPECT_NEAR(yMax - yMin, 38.0, 1e-3);
    EXPECT_NEAR(zMax - zMin, 28.0, 1e-3);

    // Volume ≈ 48 × 38 × 28 = 51072 mm³.
    GProp_GProps p;
    BRepGProp::VolumeProperties(out.workpiece->shape(), p);
    EXPECT_NEAR(p.Mass(), 48.0 * 38.0 * 28.0, 1.0);
}
