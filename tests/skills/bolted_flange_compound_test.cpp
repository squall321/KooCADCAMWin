// @lat: [[process/test-strategy#skill round-trip]]
//
// bolted_flange_compound skill — REAL geometric verification (not metadata
// clone). 5-test sweep covering:
//   1. apply() produces non-null shape with expected volume delta
//   2. validate() rejects bad input (non-standard bolt count) + apply throws
//   3. signature flags is_compound=true and subfeature_count>=2 (4 + N bolts)
//   4. recognize() returns a candidate with confidence > 0.5
//   5. feature-specific: face count grew by ≥ (4 + bolt_count)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bolted_flange_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

skill::bolted_flange_compound::Input goodInput()
{
    skill::bolted_flange_compound::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 0.0;
    in.center_y_mm           = 0.0;
    in.axis_dir              = gp_Dir(0, 0, 1);
    in.flange_outer_dia_mm   = 200.0;
    in.flange_thickness_mm   = 22.0;
    in.pipe_bore_dia_mm      = 100.0;
    in.raised_face_dia_mm    = 142.0;
    in.raised_face_height_mm = 2.0;
    in.gasket_groove_id_mm   = 122.0;
    in.gasket_groove_od_mm   = 130.0;
    in.gasket_groove_depth_mm = 1.5;
    in.bolt_count            = 8;
    in.bolt_circle_dia_mm    = 170.0;
    in.bolt_hole_dia_mm      = 22.0;
    return in;
}

}  // namespace

// ─── 1. Apply produces REAL volume delta matching expected geometry ───────
TEST(SkillBoltedFlangeCompound, ApplyProducesExpectedVolumeDelta)
{
    // Thin pad below flange — flange grows on top via fuse.
    auto stock = skill::createCuboidStock(220.0, 220.0, 4.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    // Center the flange on the stock top — stock spans (0..220, 0..220, 0..4).
    in.center_x_mm = 110.0;
    in.center_y_mm = 110.0;

    auto out = skill::bolted_flange_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Expected ADDED volume = disc + raised face
    const double Rdisc = in.flange_outer_dia_mm / 2.0;
    const double Rrf   = in.raised_face_dia_mm  / 2.0;
    const double vAdded = M_PI * Rdisc * Rdisc * in.flange_thickness_mm
                        + M_PI * Rrf   * Rrf   * in.raised_face_height_mm;
    // Expected REMOVED volume = bore + groove + N bolts
    const double Rbore = in.pipe_bore_dia_mm / 2.0;
    const double Rgo   = in.gasket_groove_od_mm / 2.0;
    const double Rgi   = in.gasket_groove_id_mm / 2.0;
    const double Rbolt = in.bolt_hole_dia_mm / 2.0;
    const double totalH = in.flange_thickness_mm + in.raised_face_height_mm;
    const double vRemoved =
        M_PI * Rbore * Rbore * totalH
        + M_PI * (Rgo * Rgo - Rgi * Rgi) * in.gasket_groove_depth_mm
        + static_cast<double>(in.bolt_count) *
          M_PI * Rbolt * Rbolt * in.flange_thickness_mm;

    const double expectedDelta = vAdded - vRemoved;
    const double volAfter = volumeOf(out.workpiece->shape());
    const double actualDelta = volAfter - volBefore;

    EXPECT_GT(actualDelta, 0.0) << "compound should add net material";
    EXPECT_NEAR(actualDelta, expectedDelta, std::abs(expectedDelta) * 0.20)
        << "actual delta " << actualDelta
        << " vs expected " << expectedDelta;

    // Face count grew (≥ N bolt holes + central bore + groove faces).
    EXPECT_GT(out.workpiece->faceCount(), facesBefore + in.bolt_count);
}

// ─── 2. Validate rejects bad input + apply throws ─────────────────────────
TEST(SkillBoltedFlangeCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(220.0, 220.0, 4.0);

    auto in = goodInput();
    in.center_x_mm = 110.0;
    in.center_y_mm = 110.0;
    in.bolt_count  = 5;  // not in {4, 8, 12, 16}

    auto r = skill::bolted_flange_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-ASME-B16.5"));
    EXPECT_THROW(skill::bolted_flange_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature is_compound=true, subfeature_count>=2 ───────────────────
TEST(SkillBoltedFlangeCompound, SignatureIsCompoundWithSubfeatures)
{
    auto stock = skill::createCuboidStock(220.0, 220.0, 4.0);

    auto in = goodInput();
    in.center_x_mm = 110.0;
    in.center_y_mm = 110.0;

    auto out = skill::bolted_flange_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bolted_flange_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    const int subCount = out.signature.pattern.value("subfeature_count", 0);
    EXPECT_GE(subCount, 2);
    EXPECT_EQ(subCount, 4 + in.bolt_count);

    // Params round-trip
    EXPECT_EQ(out.signature.params.value("bolt_count", -1), in.bolt_count);
    EXPECT_NEAR(
        out.signature.params.value("flange_outer_dia_mm", 0.0),
        in.flange_outer_dia_mm, 1e-6);
}

// ─── 4. Recognize finds geometric candidate with confidence > 0.5 ─────────
TEST(SkillBoltedFlangeCompound, RecognizeFindsCompound)
{
    auto stock = skill::createCuboidStock(220.0, 220.0, 4.0);

    auto in = goodInput();
    in.center_x_mm = 110.0;
    in.center_y_mm = 110.0;

    auto out = skill::bolted_flange_compound::apply(*stock, in);
    auto cands = skill::bolted_flange_compound::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty()) << "expected ≥ 1 recognition candidate";
    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence > 0.5) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

// ─── 5. Feature-specific: bolt-count recovery accuracy ────────────────────
TEST(SkillBoltedFlangeCompound, RecoveredBoltCountMatches)
{
    auto stock = skill::createCuboidStock(220.0, 220.0, 4.0);

    auto in = goodInput();
    in.center_x_mm = 110.0;
    in.center_y_mm = 110.0;
    in.bolt_count  = 4;        // smaller pattern
    in.bolt_hole_dia_mm = 18.0;

    auto out = skill::bolted_flange_compound::apply(*stock, in);
    auto cands = skill::bolted_flange_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    // At least one candidate (geometric or replay) should recover the bolt
    // count and bolt-circle dia.
    bool found = false;
    for (const auto& c : cands) {
        if (c.recovered_params.value("bolt_count", -1) == in.bolt_count &&
            std::abs(c.recovered_params.value("bolt_circle_dia_mm", 0.0) -
                     in.bolt_circle_dia_mm) < 4.0) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "recognized bolt_count or PCD did not match";
}
