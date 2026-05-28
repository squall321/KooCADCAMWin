// @lat: [[process/test-strategy#skill round-trip]]
//
// first_article_inspect — FAI metadata-only skill.  Covers:
//   1. apply leaves geometry unchanged (clone-only)
//   2. validate rejects bad input (empty approval, bad counts)
//   3. signature records kind + features_checked + dimensions_pass/_total
//   4. recognize replays from metadata
//   5. specific: fully_approved flag + incomplete FAI info finding

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/first_article_inspect.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillFirstArticleInspect, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double   volBefore  = volumeOf(stock->shape());
    const int      faceBefore = stock->faceCount();
    const int      edgeBefore = stock->edgeCount();

    skill::first_article_inspect::Input in;
    in.features_checked   = 8;
    in.dimensions_pass    = 18;
    in.dimensions_total   = 18;
    in.approval_signature = "INSP-101";
    in.inspector_name     = "Park";

    auto out = skill::first_article_inspect::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillFirstArticleInspect, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Empty approval signature → error.
    skill::first_article_inspect::Input in;
    in.features_checked   = 3;
    in.dimensions_pass    = 5;
    in.dimensions_total   = 5;
    in.approval_signature = "";    // INVALID
    auto r = skill::first_article_inspect::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FAI-APPROVAL"));
    EXPECT_THROW(skill::first_article_inspect::apply(*stock, in),
                 skill::SkillError);

    // dimensions_pass > dimensions_total → error.
    skill::first_article_inspect::Input in2;
    in2.features_checked   = 3;
    in2.dimensions_pass    = 10;
    in2.dimensions_total   = 5;
    in2.approval_signature = "INSP-1";
    auto r2 = skill::first_article_inspect::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    // dimensions_total <= 0 → error.
    skill::first_article_inspect::Input in3;
    in3.features_checked   = 1;
    in3.dimensions_pass    = 0;
    in3.dimensions_total   = 0;
    in3.approval_signature = "INSP-1";
    auto r3 = skill::first_article_inspect::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillFirstArticleInspect, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::first_article_inspect::Input in;
    in.features_checked   = 12;
    in.dimensions_pass    = 24;
    in.dimensions_total   = 24;
    in.approval_signature = "QA-OFFICER-7";
    in.inspector_name     = "Kim";

    auto out = skill::first_article_inspect::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id,
              std::string("first_article_inspect"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("first_article_inspect"));
    EXPECT_EQ(out.signature.pattern["features_checked"].get<int>(), 12);
    EXPECT_EQ(out.signature.pattern["dimensions_pass"].get<int>(),  24);
    EXPECT_EQ(out.signature.pattern["dimensions_total"].get<int>(), 24);
    EXPECT_EQ(out.signature.params["approval_signature"].get<std::string>(),
              std::string("QA-OFFICER-7"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillFirstArticleInspect, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Raw stock — no FAI yet — recognize is empty.
    auto cands0 = skill::first_article_inspect::recognize(*stock);
    EXPECT_TRUE(cands0.empty());

    skill::first_article_inspect::Input in;
    in.features_checked   = 4;
    in.dimensions_pass    = 9;
    in.dimensions_total   = 10;
    in.approval_signature = "INSP-202";
    auto out = skill::first_article_inspect::apply(*stock, in);

    auto cands = skill::first_article_inspect::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("first_article_inspect"));
    EXPECT_EQ(cands[0].recovered_params["approval_signature"]
                  .get<std::string>(),
              std::string("INSP-202"));
    EXPECT_EQ(cands[0].recovered_params["dimensions_pass"].get<int>(), 9);

    // Workpiece without metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::first_article_inspect::recognize(raw).empty());
}

// ─── 5. Specific: fully_approved flag + incomplete-FAI info-level ────────
TEST(SkillFirstArticleInspect, FullyApprovedFlagAndIncompleteInfo)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // pass == total → fully_approved, no DFM-FAI-INCOMPLETE.
    skill::first_article_inspect::Input full;
    full.features_checked   = 2;
    full.dimensions_pass    = 6;
    full.dimensions_total   = 6;
    full.approval_signature = "INSP-303";
    auto rFull = skill::first_article_inspect::validate(*stock, full);
    EXPECT_TRUE(rFull.passed);
    EXPECT_FALSE(hasFinding(rFull, "DFM-FAI-INCOMPLETE"));
    auto outFull = skill::first_article_inspect::apply(*stock, full);
    EXPECT_TRUE(outFull.signature.pattern["fully_approved"].get<bool>());

    // pass < total → DFM-FAI-INCOMPLETE info, still passes.
    skill::first_article_inspect::Input partial;
    partial.features_checked   = 2;
    partial.dimensions_pass    = 3;
    partial.dimensions_total   = 6;
    partial.approval_signature = "INSP-303";
    auto rPart = skill::first_article_inspect::validate(*stock, partial);
    EXPECT_TRUE(rPart.passed);
    EXPECT_TRUE(hasFinding(rPart, "DFM-FAI-INCOMPLETE"));
    auto outPart = skill::first_article_inspect::apply(*stock, partial);
    EXPECT_FALSE(outPart.signature.pattern["fully_approved"].get<bool>());

    // pass_rate is recorded correctly.
    EXPECT_NEAR(outPart.signature.pattern["pass_rate"].get<double>(),
                0.5, 1e-9);
}
