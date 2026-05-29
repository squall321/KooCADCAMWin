// @lat: [[process/test-strategy#skill round-trip]]
//
// waveguide_pattern skill — photonic-IC waveguide etch.  Covers:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput  (empty mask_id → DFM error)
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific assertion — pattern.aspect_ratio = depth/width

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/waveguide_pattern.hpp"

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

// ─── 1. Apply: clones geometry unchanged ──────────────────────────────────
TEST(SkillWaveguidePattern, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::waveguide_pattern::Input in;
    in.mask_id       = "WG_SOI_v1";
    in.etch_depth_nm = 220.0;
    in.line_width_nm = 500.0;
    in.process       = "ICP";

    auto out = skill::waveguide_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("waveguide_pattern"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (empty mask_id) ────────────────────────
TEST(SkillWaveguidePattern, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5);

    skill::waveguide_pattern::Input in;
    in.mask_id       = "";              // empty — invalid
    in.etch_depth_nm = 220.0;
    in.line_width_nm = 500.0;
    in.process       = "ICP";

    auto r = skill::waveguide_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::waveguide_pattern::apply(*stock, in), skill::SkillError);

    // Out-of-band depth → DFM error.
    skill::waveguide_pattern::Input badDepth;
    badDepth.mask_id       = "WG_X";
    badDepth.etch_depth_nm = 10.0;       // < 50 nm bound
    badDepth.line_width_nm = 500.0;
    auto r2 = skill::waveguide_pattern::validate(*stock, badDepth);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-WG-DEPTH"));
}

// ─── 3. Signature records kind + params ───────────────────────────────────
TEST(SkillWaveguidePattern, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5);

    skill::waveguide_pattern::Input in;
    in.mask_id       = "WG_RING_v2";
    in.etch_depth_nm = 250.0;
    in.line_width_nm = 480.0;
    in.process       = "ICP";

    auto out = skill::waveguide_pattern::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("waveguide_pattern"));
    EXPECT_EQ(out.signature.pattern["mask_id"].get<std::string>(),
              std::string("WG_RING_v2"));
    EXPECT_NEAR(out.signature.pattern["etch_depth_nm"].get<double>(),
                250.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["line_width_nm"].get<double>(),
                480.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["process"].get<std::string>(),
              std::string("ICP"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("ICP_etcher"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillWaveguidePattern, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5);

    skill::waveguide_pattern::Input in;
    in.mask_id       = "WG_SOI_v1";
    in.etch_depth_nm = 220.0;
    in.line_width_nm = 500.0;
    in.process       = "ICP";
    auto out = skill::waveguide_pattern::apply(*stock, in);

    auto cands = skill::waveguide_pattern::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["mask_id"].get<std::string>(),
              std::string("WG_SOI_v1"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::waveguide_pattern::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "waveguide_pattern cannot be recognized geometrically";
}

// ─── 5. Specific assertion — aspect_ratio = etch_depth / line_width ───────
TEST(SkillWaveguidePattern, PatternRecordsAspectRatio)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 0.5);

    skill::waveguide_pattern::Input in;
    in.mask_id       = "WG_DEEP";
    in.etch_depth_nm = 1000.0;
    in.line_width_nm = 200.0;       // aspect = 5.0
    in.process       = "DRIE";

    auto out = skill::waveguide_pattern::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["aspect_ratio"].get<double>(),
                5.0, 1e-9);
}
