// @lat: [[process/test-strategy#skill round-trip]]
//
// fiber_splice skill — fusion splice metadata.  Covers:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput  (negative loss_db → DFM error)
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific assertion — laser method changes tool_type & material

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fiber_splice.hpp"

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
TEST(SkillFiberSplice, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0, "aluminum_6061");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::fiber_splice::Input in;
    in.loss_db       = 0.05;
    in.splice_method = "arc";

    auto out = skill::fiber_splice::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("fiber_splice"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (negative loss) ────────────────────────
TEST(SkillFiberSplice, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::fiber_splice::Input in;
    in.loss_db       = -0.1;          // negative — invalid (loss is ≥ 0)
    in.splice_method = "arc";

    auto r = skill::fiber_splice::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::fiber_splice::apply(*stock, in), skill::SkillError);

    // Also reject runaway loss as a DFM error.
    skill::fiber_splice::Input bad = in;
    bad.loss_db = 2.5;                // > 1.0 dB
    auto r2 = skill::fiber_splice::validate(*stock, bad);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-SPLICE-LOSS-HIGH"));
}

// ─── 3. Signature records kind + params ───────────────────────────────────
TEST(SkillFiberSplice, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::fiber_splice::Input in;
    in.loss_db       = 0.08;
    in.splice_method = "arc";

    auto out = skill::fiber_splice::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fiber_splice"));
    EXPECT_NEAR(out.signature.pattern["loss_db"].get<double>(), 0.08, 1e-9);
    EXPECT_EQ(out.signature.pattern["splice_method"].get<std::string>(),
              std::string("arc"));
    EXPECT_TRUE(out.signature.pattern["protected"].get<bool>())
        << "arc-fusion field splice is sleeve-protected by default";
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("arc_fusion_splicer"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFiberSplice, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::fiber_splice::Input in;
    in.loss_db       = 0.05;
    in.splice_method = "arc";
    auto out = skill::fiber_splice::apply(*stock, in);

    auto cands = skill::fiber_splice::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["splice_method"].get<std::string>(),
              std::string("arc"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::fiber_splice::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "fiber_splice cannot be recognized geometrically";
}

// ─── 5. Specific assertion — laser method updates tool_type & material ───
TEST(SkillFiberSplice, LaserMethodUpdatesTooling)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 5.0);

    skill::fiber_splice::Input in;
    in.loss_db       = 0.04;
    in.splice_method = "laser";

    auto out = skill::fiber_splice::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("co2_laser_splicer"));
    EXPECT_EQ(out.signature.tooling.tool_material,
              std::string("co2_laser_optics"));
    EXPECT_EQ(out.signature.pattern["splice_method"].get<std::string>(),
              std::string("laser"));
}
