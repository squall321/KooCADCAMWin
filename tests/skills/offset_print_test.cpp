// @lat: [[process/test-strategy#skill round-trip]]
//
// offset_print — lithographic offset printing.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. HeavyPaperEmitsInfoFinding

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/offset_print.hpp"

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillOffsetPrint, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.1);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::offset_print::Input in;
    in.paper_gsm = 120.0;
    in.ink_set   = 4;
    in.sheets    = 5000;

    auto out = skill::offset_print::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("offset_print"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillOffsetPrint, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 0.1);

    skill::offset_print::Input badGsm;
    badGsm.paper_gsm = 0.0;
    badGsm.ink_set   = 4;
    badGsm.sheets    = 500;
    {
        auto r = skill::offset_print::validate(*stock, badGsm);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::offset_print::apply(*stock, badGsm), skill::SkillError);

    skill::offset_print::Input badInk;
    badInk.paper_gsm = 120.0;
    badInk.ink_set   = 0;
    badInk.sheets    = 500;
    {
        auto r = skill::offset_print::validate(*stock, badInk);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::offset_print::apply(*stock, badInk), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillOffsetPrint, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.1);

    skill::offset_print::Input in;
    in.paper_gsm = 200.0;
    in.ink_set   = 6;        // hexachrome
    in.sheets    = 10000;

    auto out = skill::offset_print::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("offset_print"));
    EXPECT_NEAR(out.signature.pattern["paper_gsm"].get<double>(), 200.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["ink_set"].get<int>(), 6);
    EXPECT_EQ(out.signature.pattern["sheets"].get<int>(), 10000);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillOffsetPrint, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.1);

    skill::offset_print::Input in;
    in.paper_gsm = 120.0;
    in.ink_set   = 4;
    in.sheets    = 1000;

    auto out   = skill::offset_print::apply(*stock, in);
    auto cands = skill::offset_print::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["ink_set"].get<int>(), 4);
    EXPECT_NEAR(cands[0].recovered_params["paper_gsm"].get<double>(),
                120.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::offset_print::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: heavy board paper emits info finding (still passes) ─────
TEST(SkillOffsetPrint, HeavyPaperEmitsInfoFinding)
{
    auto stock = skill::createCuboidStock(297.0, 210.0, 0.4);

    skill::offset_print::Input in;
    in.paper_gsm = 500.0;       // > 400 → heavy
    in.ink_set   = 4;
    in.sheets    = 100;

    auto r = skill::offset_print::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PAPER-HEAVY"));
    EXPECT_NO_THROW(skill::offset_print::apply(*stock, in));
}
