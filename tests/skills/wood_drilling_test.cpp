// @lat: [[process/test-strategy#skill round-trip]]
//
// wood_drilling skill — drill hole in wood with bit-specific DFM envelope.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wood_drilling.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ─── 1. Apply handles input: 10mm board drilled to 8mm with 6mm hole ─────
TEST(SkillWoodDrilling, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0, "oak");

    skill::wood_drilling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 50.0;
    in.position_y_mm = 50.0;
    in.axis_dir      = gp_Dir(0, 0, -1);
    in.diameter_mm   = 6.0;
    in.depth_mm      = 8.0;
    in.bit_type      = "brad_point";

    auto out = skill::wood_drilling::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double approx = M_PI * 3.0 * 3.0 * 8.0;
    const double diff = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(diff, approx, approx * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("wood_drilling"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input (too small / bad bit / no depth) ──────
TEST(SkillWoodDrilling, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);

    // < 3 mm → DFM-WOOD-DIA-MIN error.
    skill::wood_drilling::Input small;
    small.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    small.position_x_mm = 50.0;
    small.position_y_mm = 50.0;
    small.diameter_mm   = 2.0;
    small.depth_mm      = 5.0;
    auto r1 = skill::wood_drilling::validate(*stock, small);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-WOOD-DIA-MIN"));
    EXPECT_THROW(skill::wood_drilling::apply(*stock, small), skill::SkillError);

    // Unknown bit_type → DFM-INPUT error.
    skill::wood_drilling::Input badBit;
    badBit.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badBit.position_x_mm = 50.0;
    badBit.position_y_mm = 50.0;
    badBit.diameter_mm   = 6.0;
    badBit.depth_mm      = 5.0;
    badBit.bit_type      = "twist";   // unsupported
    auto r2 = skill::wood_drilling::validate(*stock, badBit);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    // Blind with depth 0 → DFM-INPUT.
    skill::wood_drilling::Input noDepth;
    noDepth.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    noDepth.position_x_mm = 50.0;
    noDepth.position_y_mm = 50.0;
    noDepth.diameter_mm   = 6.0;
    noDepth.depth_mm      = 0.0;
    noDepth.through_hole  = false;
    auto r3 = skill::wood_drilling::validate(*stock, noDepth);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + bit_type + diameter ─────────────────────
TEST(SkillWoodDrilling, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);

    skill::wood_drilling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 40.0;
    in.diameter_mm   = 12.0;
    in.depth_mm      = 10.0;
    in.bit_type      = "forstner";

    auto out = skill::wood_drilling::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("wood_drilling"));
    EXPECT_EQ(out.signature.pattern["bit_type"].get<std::string>(), std::string("forstner"));
    EXPECT_NEAR(out.signature.pattern["diameter_mm"].get<double>(), 12.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["cylindrical_face_count"].get<int>(), 1);
    EXPECT_EQ(out.signature.pattern["circular_edge_count"].get<int>(), 2);
    EXPECT_TRUE(out.signature.pattern["bottom_planar_face_present"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("wood_drill_bit"));
}

// ─── 4. Recognize via metadata replay (plus geometric candidate) ─────────
TEST(SkillWoodDrilling, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);

    skill::wood_drilling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 50.0;
    in.position_y_mm = 50.0;
    in.diameter_mm   = 8.0;
    in.depth_mm      = 10.0;
    in.bit_type      = "auger";
    auto out = skill::wood_drilling::apply(*stock, in);

    auto cands = skill::wood_drilling::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    // Metadata candidate present (confidence 1.0).
    bool sawMetaCandidate = false;
    for (const auto& c : cands) {
        if (c.confidence >= 0.99) {
            sawMetaCandidate = true;
            EXPECT_EQ(c.recovered_params["bit_type"].get<std::string>(), std::string("auger"));
            EXPECT_NEAR(c.recovered_params["diameter_mm"].get<double>(), 8.0, 1e-3);
        }
    }
    EXPECT_TRUE(sawMetaCandidate);

    // Strip metadata; geometric candidate should still appear (cylindrical bore).
    skill::Workpiece bare(out.workpiece->shape(), out.workpiece->material());
    auto cands2 = skill::wood_drilling::recognize(bare);
    ASSERT_FALSE(cands2.empty());
    EXPECT_NEAR(cands2[0].recovered_params["diameter_mm"].get<double>(), 8.0, 1e-2);
    EXPECT_LT(cands2[0].confidence, 0.99);   // not metadata-confident
    EXPECT_GT(cands2[0].confidence, 0.3);    // but a real geometric match
}

// ─── 5. end_grain hint emits info-level finding but does NOT block ────────
TEST(SkillWoodDrilling, EndGrainInfoDoesNotBlock)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 15.0);

    skill::wood_drilling::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 50.0;
    in.position_y_mm = 50.0;
    in.diameter_mm   = 6.0;
    in.depth_mm      = 5.0;
    in.end_grain     = true;

    auto r = skill::wood_drilling::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "end_grain is INFO, not error";
    EXPECT_TRUE(hasFinding(r, "DFM-WOOD-ENDGRAIN"));
    EXPECT_NO_THROW(skill::wood_drilling::apply(*stock, in));
}
