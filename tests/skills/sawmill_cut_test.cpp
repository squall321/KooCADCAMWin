// @lat: [[process/test-strategy#skill round-trip]]
//
// sawmill_cut skill — bulk lumber cut (rip / cross-cut / resaw).
// Mirrors the rolling pattern (rebuild workpiece smaller).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sawmill_cut.hpp"

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

// ─── 1. Apply handles a rip cut: shrink X, preserve Y/Z ────────────────────
TEST(SkillSawmillCut, ApplyHandlesRipCut)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 40.0, "oak");

    skill::sawmill_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cut_type         = "rip";
    in.cut_thickness_mm = 80.0;       // shrink along X
    in.kerf_mm          = 4.0;

    auto out = skill::sawmill_cut::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_NEAR(xMax - xMin, 80.0, 1e-6);
    EXPECT_NEAR(yMax - yMin, 100.0, 1e-6);
    EXPECT_NEAR(zMax - zMin, 40.0,  1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("sawmill_cut"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_LT(volumeOf(out.workpiece->shape()), volumeOf(stock->shape()));
}

// ─── 2. Validate rejects bad input (cut_thickness_mm >= current) ──────────
TEST(SkillSawmillCut, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 40.0);

    // Negative thickness → DFM-INPUT.
    skill::sawmill_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cut_type         = "rip";
    in.cut_thickness_mm = -5.0;
    in.kerf_mm          = 4.0;
    auto r1 = skill::sawmill_cut::validate(*stock, in);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-INPUT"));

    // target_thickness >= current bbox along axis → DFM-SAWMILL-GROWTH.
    skill::sawmill_cut::Input grow;
    grow.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    grow.cut_type         = "rip";
    grow.cut_thickness_mm = 250.0;
    grow.kerf_mm          = 4.0;
    auto r2 = skill::sawmill_cut::validate(*stock, grow);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SAWMILL-GROWTH"));
    EXPECT_THROW(skill::sawmill_cut::apply(*stock, grow), skill::SkillError);

    // unknown cut_type → DFM-INPUT (error).
    skill::sawmill_cut::Input bad;
    bad.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.cut_type         = "rotary";
    bad.cut_thickness_mm = 50.0;
    bad.kerf_mm          = 4.0;
    auto r3 = skill::sawmill_cut::validate(*stock, bad);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + cut_type + cut_thickness_mm ──────────────
TEST(SkillSawmillCut, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(150.0, 60.0, 30.0);

    skill::sawmill_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cut_type         = "cross_cut";
    in.cut_thickness_mm = 40.0;       // shrink Y to 40
    in.kerf_mm          = 5.0;

    auto out = skill::sawmill_cut::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("sawmill_cut"));
    EXPECT_EQ(out.signature.pattern["cut_type"].get<std::string>(), std::string("cross_cut"));
    EXPECT_NEAR(out.signature.pattern["cut_thickness_mm"].get<double>(), 40.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["kerf_mm"].get<double>(), 5.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["cut_axis_index"].get<int>(), 1);
    EXPECT_EQ(out.signature.pattern["planar_face_count"].get<int>(), 6);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("circular_saw"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSawmillCut, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 40.0);

    skill::sawmill_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cut_type         = "resaw";
    in.cut_thickness_mm = 20.0;
    in.kerf_mm          = 3.5;
    auto out = skill::sawmill_cut::apply(*stock, in);

    auto cands = skill::sawmill_cut::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["cut_type"].get<std::string>(), std::string("resaw"));
    EXPECT_NEAR(cands[0].recovered_params["cut_thickness_mm"].get<double>(), 20.0, 1e-6);

    // Metadata-free workpiece → geometric fallback (low confidence).
    skill::Workpiece bare(out.workpiece->shape(), out.workpiece->material());
    auto cands2 = skill::sawmill_cut::recognize(bare);
    ASSERT_FALSE(cands2.empty());
    EXPECT_LT(cands2[0].confidence, 0.5);
}

// ─── 5. Resaw cut along Z reduces thickness only ──────────────────────────
TEST(SkillSawmillCut, ResawShrinkOnlyZAxis)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 50.0);

    skill::sawmill_cut::Input in;
    in.entry_face       = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.cut_type         = "resaw";
    in.cut_thickness_mm = 25.0;       // shrink Z
    in.kerf_mm          = 3.0;

    auto out = skill::sawmill_cut::apply(*stock, in);
    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_NEAR(xMax - xMin, 100.0, 1e-6);   // X unchanged
    EXPECT_NEAR(yMax - yMin, 100.0, 1e-6);   // Y unchanged
    EXPECT_NEAR(zMax - zMin,  25.0, 1e-6);   // Z reduced
    EXPECT_EQ(out.signature.pattern["cut_axis_index"].get<int>(), 2);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("bandsaw"));
}
