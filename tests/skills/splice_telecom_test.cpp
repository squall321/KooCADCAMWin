// @lat: [[process/test-strategy#skill round-trip]]
//
// splice_telecom — 5-case sweep: identity-geometry synthesis, DFM gates on
// splice_count / loss / method, metadata replay, total-loss arithmetic
// specific case.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/splice_telecom.hpp"

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

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillSpliceTelecom, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::splice_telecom::Input in;
    in.splice_count       = 4;
    in.loss_db_per_splice = 0.08;
    in.splice_method      = "fusion";

    auto out = skill::splice_telecom::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("splice_telecom"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: zero count, negative loss, empty method rejected ─────────────
TEST(SkillSpliceTelecom, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::splice_telecom::Input bad;
    bad.splice_count       = 0;                // invalid
    bad.loss_db_per_splice = 0.05;
    bad.splice_method      = "fusion";
    EXPECT_FALSE(skill::splice_telecom::validate(*stock, bad).passed);
    EXPECT_THROW(skill::splice_telecom::apply(*stock, bad), skill::SkillError);

    bad.splice_count       = 3;
    bad.loss_db_per_splice = -0.1;             // invalid
    EXPECT_FALSE(skill::splice_telecom::validate(*stock, bad).passed);

    bad.loss_db_per_splice = 0.05;
    bad.splice_method      = "";               // invalid
    EXPECT_FALSE(skill::splice_telecom::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + count/loss/method ────────────────────────
TEST(SkillSpliceTelecom, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::splice_telecom::Input in;
    in.splice_count       = 6;
    in.loss_db_per_splice = 0.12;
    in.splice_method      = "mechanical";

    auto out = skill::splice_telecom::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("splice_telecom"));
    EXPECT_EQ(out.signature.pattern["splice_count"].get<int>(),    6);
    EXPECT_NEAR(out.signature.pattern["loss_db_per_splice"].get<double>(),
                0.12, 1e-9);
    EXPECT_EQ(out.signature.pattern["splice_method"].get<std::string>(),
              std::string("mechanical"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    // mechanical method → splice_kit tool type
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("splice_kit"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSpliceTelecom, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::splice_telecom::Input in;
    in.splice_count       = 2;
    in.loss_db_per_splice = 0.05;
    in.splice_method      = "fusion";

    auto out = skill::splice_telecom::apply(*stock, in);
    auto cands = skill::splice_telecom::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["splice_count"].get<int>(), 2);
    EXPECT_EQ(cands[0].recovered_params["splice_method"].get<std::string>(),
              std::string("fusion"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::splice_telecom::recognize(raw).empty());
}

// ── 5. SPECIFIC: total_loss_db = count × loss_db_per_splice ──────────────
TEST(SkillSpliceTelecom, TotalLossEqualsCountTimesLossPerSplice)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::splice_telecom::Input in;
    in.splice_count       = 8;
    in.loss_db_per_splice = 0.04;
    in.splice_method      = "fusion";

    auto out = skill::splice_telecom::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["total_loss_db"].get<double>(),
                8.0 * 0.04, 1e-9);

    // High per-splice loss triggers SPLICE-LOSS warning, still passes.
    skill::splice_telecom::Input hi;
    hi.splice_count       = 2;
    hi.loss_db_per_splice = 0.8;          // > 0.5 dB threshold
    hi.splice_method      = "mechanical";
    auto r = skill::splice_telecom::validate(*stock, hi);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-SPLICE-LOSS"));
}
